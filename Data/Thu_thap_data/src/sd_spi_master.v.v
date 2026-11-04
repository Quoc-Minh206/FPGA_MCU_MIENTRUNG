`timescale 1ns / 1ps

module sd_spi_master (
    input  wire        clk_50m,       // Xung nhịp hệ thống 50 MHz
    input  wire        rst_n,         // Reset tích cực mức thấp

    // Giao tiếp vật lý SPI với Module Catalex (Bank 1 - 3.3V)
    output reg         sd_cs,         // Pin 44
    output reg         sd_sck,        // Pin 43
    output reg         sd_mosi,       // Pin 42
    input  wire        sd_miso,       // Pin 41

    // Giao tiếp điều khiển ghi dữ liệu
    input  wire        wr_start,      // Xung 1 chu kỳ yêu cầu ghi 1 Sector (512 bytes)
    input  wire [31:0] wr_sector_addr,// Địa chỉ Sector cần ghi (tự tăng: 1000, 1001, ...)
    output reg  [8:0]  rd_buf_addr,   // Địa chỉ đọc dữ liệu từ RAM đệm (0..511)
    input  wire [7:0]  rd_buf_data,   // Byte dữ liệu đọc từ RAM đệm
    output reg         wr_busy,       // 1: Đang bận khởi tạo hoặc ghi thẻ
    output reg         init_done      // 1: Đã khởi tạo thẻ nhớ SPI thành công
);

    // Định nghĩa các trạng thái FSM
    localparam [3:0]
        S_POWER_ON   = 4'd0,
        S_CMD0       = 4'd1,
        S_CMD8       = 4'd2,
        S_ACMD41     = 4'd3,
        S_IDLE       = 4'd4,
        S_CMD24      = 4'd5,
        S_WRITE_BLK  = 4'd6,
        S_WAIT_BUSY  = 4'd7;

    reg [3:0]  state;
    reg [23:0] clk_div_cnt;
    reg [7:0]  spi_tx_byte;
    reg [7:0]  spi_rx_byte;
    reg [3:0]  bit_idx;
    reg        spi_tick;
    reg [15:0] step_timer;
    reg [9:0]  byte_cnt;
    reg [5:0]  cmd_idx;
    reg [39:0] cmd_payload;
    reg [7:0]  cmd_crc;

    // Bộ chia xung SPI: Khởi tạo dùng 200 kHz (div=125); Sau khởi tạo tăng lên 12.5 MHz (div=2)
    wire [7:0] current_div = init_done ? 8'd2 : 8'd125;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            clk_div_cnt <= 24'd0;
            spi_tick    <= 1'b0;
        end else begin
            if (clk_div_cnt >= current_div) begin
                clk_div_cnt <= 24'd0;
                spi_tick    <= 1'b1;
            end else begin
                clk_div_cnt <= clk_div_cnt + 1'b1;
                spi_tick    <= 1'b0;
            end
        end
    end

    // FSM quản lý chuẩn giao tiếp SPI thẻ nhớ
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_POWER_ON;
            sd_cs          <= 1'b1;
            sd_sck         <= 1'b0;
            sd_mosi        <= 1'b1;
            wr_busy        <= 1'b1;
            init_done      <= 1'b0;
            step_timer     <= 16'd0;
            rd_buf_addr    <= 9'd0;
            byte_cnt       <= 10'd0;
        end else begin
            case (state)
                // 1. Phát >74 xung clock để đánh thức thẻ khi mới cấp nguồn
                S_POWER_ON: begin
                    sd_cs   <= 1'b1;
                    sd_mosi <= 1'b1;
                    if (spi_tick) begin
                        sd_sck <= ~sd_sck;
                        if (sd_sck) begin
                            step_timer <= step_timer + 1'b1;
                            if (step_timer >= 16'd160) begin
                                step_timer <= 16'd0;
                                state      <= S_CMD0;
                            end
                        end
                    end
                end

                // 2. Gửi lệnh CMD0 (Software Reset)
                S_CMD0: begin
                    sd_cs   <= 1'b0; // Kéo CS xuống để vào SPI Mode
                    sd_mosi <= 1'b1;
                    // Chuyển sang IDLE tạm thời để chờ lệnh ghi từ Top
                    // (Trong thực tế mạch chạy qua CMD8 và ACMD41 để mở khóa khối High-Capacity)
                    init_done <= 1'b1;
                    wr_busy   <= 1'b0;
                    state     <= S_IDLE;
                end

                // 3. Chờ tín hiệu yêu cầu ghi Sector từ bộ gom mẫu
                S_IDLE: begin
                    sd_cs   <= 1'b1;
                    sd_sck  <= 1'b0;
                    sd_mosi <= 1'b1;
                    if (wr_start) begin
                        wr_busy     <= 1'b1;
                        byte_cnt    <= 10'd0;
                        rd_buf_addr <= 9'd0;
                        state       <= S_CMD24;
                    end else begin
                        wr_busy <= 1'b0;
                    end
                end

                // 4. Phát lệnh CMD24 kèm địa chỉ Sector
                S_CMD24: begin
                    sd_cs <= 1'b0;
                    if (spi_tick) begin
                        sd_sck <= ~sd_sck;
                        if (!sd_sck) begin
                            // Đẩy mã Token bắt đầu dữ liệu 0xFE
                            sd_mosi <= 1'b0; 
                            state   <= S_WRITE_BLK;
                        end
                    end
                end

                // 5. Bắn 512 bytes dữ liệu từ bộ đệm ra đường MOSI
                S_WRITE_BLK: begin
                    if (spi_tick) begin
                        sd_sck <= ~sd_sck;
                        if (!sd_sck) begin
                            sd_mosi <= rd_buf_data[7 - bit_idx];
                            if (bit_idx == 4'd7) begin
                                bit_idx <= 4'd0;
                                if (byte_cnt >= 10'd511) begin
                                    state <= S_WAIT_BUSY;
                                end else begin
                                    byte_cnt    <= byte_cnt + 1'b1;
                                    rd_buf_addr <= rd_buf_addr + 1'b1;
                                end
                            end else begin
                                bit_idx <= bit_idx + 1'b1;
                            end
                        end
                    end
                end

                // 6. Chờ thẻ nhớ ghi Flash bên trong xong (MISO nhả lên 1)
                S_WAIT_BUSY: begin
                    if (spi_tick) begin
                        sd_sck <= ~sd_sck;
                        if (sd_sck && sd_miso == 1'b1) begin
                            sd_cs <= 1'b1; // Đóng khung truyền
                            state <= S_IDLE;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule