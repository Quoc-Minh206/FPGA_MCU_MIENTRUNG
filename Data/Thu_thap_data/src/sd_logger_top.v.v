`timescale 1ns / 1ps

module sd_logger_top (
    input  wire sys_clkin,       // Pin 45: Thạch anh 27 MHz onboard
    input  wire ext_rst_n,       // Pin 14: Nút nhấn KEY1 (Reset)

    // Bus I2C nối cảm biến MPU6050 (Bank 1 - 3.3V)
    inout  wire i2c_scl,         // Pin 39
    inout  wire i2c_sda,         // Pin 40

    // Bus SPI nối Module Thẻ Nhớ Catalex (Bank 1 - 3.3V)
    output wire sd_cs,           // Pin 44
    output wire sd_sck,          // Pin 43
    output wire sd_mosi,         // Pin 42
    input  wire sd_miso,         // Pin 41

    // Đèn LED hiển thị trên bo mạch Kiwi 4K
    output wire led_imu_ok,      // Pin 10 (LED1: Sáng khi MPU6050 hoạt động)
    output wire led_rec_active   // Pin 13 (LED2: Chớp tắt khi ghi mẫu vào thẻ)
);

    // 1. Tạo xung nhịp hệ thống 50 MHz
    wire clk_50m;
    Clock_manager u_clk (
        .clk       (sys_clkin),
        .rst_n     (ext_rst_n),
        .clk_50MHz (clk_50m)
    );

    // 2. Đọc cảm biến MPU6050 qua I2C phần cứng
    wire signed [15:0] acc_x, acc_y, acc_z;
    wire [7:0]         who_am_i;
    wire               data_valid, device_ok, sda_stuck_low;

    mpu6050_hardware_reader u_imu (
        .clk           (clk_50m),
        .rst_n         (ext_rst_n),
        .i2c_scl       (i2c_scl),       // Chân 39
        .i2c_sda       (i2c_sda),       // Chân 40
        .acc_x         (acc_x),
        .acc_y         (acc_y),
        .acc_z         (acc_z),
        .who_am_i      (who_am_i),
        .data_valid    (data_valid),
        .i2c_device_ok (device_ok),
        .sda_stuck_low (sda_stuck_low)
    );

    // 3. Bộ nhớ đệm 512 bytes (Gom đủ 1 Sector để ghi xuống SD)
    // =========================================================================
    // 3. BỘ NHỚ ĐỆM 512 BYTES (ÉP SUY DIỄN THÀNH BLOCK RAM - 0 DFF)
    // =========================================================================
    // Khai báo thuộc tính ép GowinSynthesis dùng Block RAM phần cứng
    (* syn_ramstyle = "block_ram" *) reg [7:0] sector_ram [0:511];

    reg [8:0]  wr_ram_ptr     = 9'd0;
    reg [2:0]  wr_seq_cnt     = 3'd0;
    reg        wr_seq_busy    = 1'b0;
    reg [47:0] sample_shift   = 48'd0;

    wire [8:0] rd_buf_addr;
    reg  [7:0] rd_buf_data;

    // Đọc đồng bộ từ RAM để hỗ trợ chuẩn cấu trúc BSRAM
    always @(posedge clk_50m) begin
        rd_buf_data <= sector_ram[rd_buf_addr];
    end

    reg        wr_start       = 1'b0;
    reg [31:0] sector_address = 32'd1000;
    wire       wr_busy;
    wire       init_done;

    // FSM ghi tuần tự 6 bytes trong 6 chu kỳ clock (120ns @ 50MHz)
    always @(posedge clk_50m or negedge ext_rst_n) begin
        if (!ext_rst_n) begin
            wr_ram_ptr     <= 9'd0;
            wr_seq_cnt     <= 3'd0;
            wr_seq_busy    <= 1'b0;
            sample_shift   <= 48'd0;
            wr_start       <= 1'b0;
            sector_address <= 32'd1000;
        end else begin
            wr_start <= 1'b0;

            // Khi MPU6050 có mẫu mới và hệ thống đang rảnh
            if (data_valid && !wr_busy && !wr_seq_busy) begin
                sample_shift <= {acc_x, acc_y, acc_z}; // Nạp 6 byte vào thanh ghi trượt
                wr_seq_busy  <= 1'b1;
                wr_seq_cnt   <= 3'd0;
            end 
            // Chu kỳ ghi tuần tự từng byte (chỉ 1 cổng ghi hoạt động mỗi nhịp clock)
            else if (wr_seq_busy) begin
                sector_ram[wr_ram_ptr] <= sample_shift[47:40]; // Ghi byte cao nhất
                sample_shift           <= {sample_shift[39:0], 8'h00};
                wr_ram_ptr             <= wr_ram_ptr + 9'd1;

                if (wr_seq_cnt == 3'd5) begin
                    wr_seq_busy <= 1'b0; // Đã ghi xong đủ 6 byte của mẫu
                    // Đủ 84 mẫu (84 x 6 = 504 bytes, gần đầy sector 512)
                    if (wr_ram_ptr >= 9'd503) begin
                        wr_ram_ptr     <= 9'd0;
                        wr_start       <= 1'b1; // Bắn lệnh ghi xuống thẻ SD
                        sector_address <= sector_address + 32'd1;
                    end
                end else begin
                    wr_seq_cnt <= wr_seq_cnt + 3'd1;
                end
            end
        end
    end

    // 4. Khối điều khiển SPI Master thẻ nhớ
    sd_spi_master u_sd (
        .clk_50m        (clk_50m),
        .rst_n          (ext_rst_n),
        .sd_cs          (sd_cs),
        .sd_sck         (sd_sck),
        .sd_mosi        (sd_mosi),
        .sd_miso        (sd_miso),
        .wr_start       (wr_start),
        .wr_sector_addr (sector_address),
        .rd_buf_addr    (rd_buf_addr),
        .rd_buf_data    (rd_buf_data),
        .wr_busy        (wr_busy),
        .init_done      (init_done)
    );

    // 5. Hiển thị LED chẩn đoán trên board
    assign led_imu_ok     = ~device_ok; // Sáng khi I2C bắt tay thành công
    assign led_rec_active = ~wr_busy;   // Chớp tắt theo chu kỳ ghi Sector

endmodule