// =============================================================================
// File:        sonix_spi_wake_controller.v
// Dự án:       Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Phân hệ:      Bộ điều khiển SPI Master & Đánh thức MCU tích hợp IP Core Gowin SPI_MASTER_Top
// Mô tả:        1. Kéo chân ngắt phần cứng mcu_wake_irq lên mức HIGH tức thời (độ trễ 20ns)
//                  khi khối toán học math_core_top phát hiện va chạm vượt ngưỡng 2.5g.
//               2. Đếm trễ micro-giây (mặc định 5 us) chờ MCU Sonix SN32F407 thức dậy
//                  khỏi chế độ Deep-Sleep và sẵn sàng module SPI Slave.
//               3. Tự động kích hoạt IP Core SPI_MASTER_Top phát xung clock SCK,
//                  đẩy trọn vẹn gói 64-bit đặc trưng {Pitch, Roll, RMS, Var} sang MCU.
//               4. Quản lý cờ spi_busy để thực hiện cơ chế Backpressure bảo toàn dữ liệu.
// =============================================================================

`timescale 1ns / 1ps

module sonix_spi_wake_controller #(
    parameter WAKE_DELAY_CYCLES = 16'd250 // 250 chu kỳ tại 50 MHz = đúng 5.0 micro-giây
)(
    input  wire                 clk,             // Xung nhịp hệ thống 50 MHz
    input  wire                 rst_n,           // Reset hệ thống tích cực mức thấp

    // Giao tiếp với khối toán học math_core_top
    input  wire                 wake_up_trigger, // Cờ vượt ngưỡng va chạm 2.5g từ accel_threshold_detector
    input  wire                 feature_valid,   // Xung báo chốt 4 đặc trưng từ math_pipeline_ctrl
    input  wire signed [15:0]   feature_pitch,   // Đặc trưng Pitch (Q8.8 độ)
    input  wire signed [15:0]   feature_roll,    // Đặc trưng Roll (Q8.8 độ)
    input  wire        [15:0]   feature_rms,     // Đặc trưng RMS^2 (Q8.8)
    input  wire        [15:0]   feature_var,     // Đặc trưng Phương sai (Q12.4)
    output reg                  spi_busy,        // Báo bận về math_core_top

    // Giao tiếp chân ngoại vi vật lý với MCU Sonix SN32F407
    output reg                  mcu_wake_irq,    // Chân ngắt GPIO đánh thức MCU tức thì (Zero-latency)
    output wire                 spi_sck,         // Chân phát xung SCK sang MCU
    output wire                 spi_mosi,        // Chân dữ liệu Master Out Slave In
    input  wire                 spi_miso,        // Chân phản hồi Master In Slave Out (Reserved)
    output wire                 spi_cs_n         // Chân Chip Select tích cực mức thấp
);

    // -------------------------------------------------------------------------
    // 1. TÍN HIỆU GIAO TIẾP VỚI IP CORE: SPI_MASTER_Top (GOWIN IP)
    // -------------------------------------------------------------------------
    reg         ip_tx_en;
    reg  [2:0]  ip_waddr;
    reg  [7:0]  ip_wdata;
    wire [7:0]  ip_rdata;
    wire        ip_spi_int;
    wire        ip_mosi;
    wire [0:0]  ip_ss_n;
    wire        ip_sclk;

    SPI_MASTER_Top u_spi_master_ip (
        .I_CLK       (clk),
        .I_RESETN    (rst_n),
        .I_TX_EN     (ip_tx_en),
        .I_WADDR     (ip_waddr),
        .I_WDATA     (ip_wdata),
        .I_RX_EN     (1'b0),
        .I_RADDR     (3'd0),
        .O_RDATA     (ip_rdata),
        .O_SPI_INT   (ip_spi_int),
        .MISO_MASTER (spi_miso),
        .MOSI_MASTER (ip_mosi),
        .SS_N_MASTER (ip_ss_n),
        .SCLK_MASTER (ip_sclk),
        .MISO_SLAVE  (),
        .MOSI_SLAVE  (1'b0),
        .SS_N_SLAVE  (1'b1),
        .SCLK_SLAVE  (1'b0)
    );

    // Kết nối ngõ ra ngoại vi trực tiếp từ IP Core
    assign spi_sck  = ip_sclk;
    assign spi_mosi = ip_mosi;
    assign spi_cs_n = ip_ss_n[0];

    // -------------------------------------------------------------------------
    // 2. MÁY TRẠNG THÁI FSM: ĐÁNH THỨC MCU & TỰ ĐỘNG ĐẨY 8 BYTE ĐẶC TRƯNG (64-BIT)
    // -------------------------------------------------------------------------
    localparam ST_IDLE       = 3'd0,
               ST_WAKE_DELAY = 3'd1,
               ST_LOAD_BYTE  = 3'd2,
               ST_WAIT_BYTE  = 3'd3,
               ST_FINISH     = 3'd4;

    reg [2:0]   state;
    reg [15:0]  us_timer;
    reg [2:0]   byte_index; // Đếm từ byte 0 đến byte 7 (8 bytes = 64 bits)
    reg [63:0]  latched_packet;
    reg [5:0]   byte_delay_cnt;

    // Phân rã gói 64-bit thành 8 byte (MSB First):
    // Byte 0,1: Pitch | Byte 2,3: Roll | Byte 4,5: RMS | Byte 6,7: Variance
    wire [7:0] current_tx_byte;
    assign current_tx_byte = (byte_index == 3'd0) ? latched_packet[63:56] :
                             (byte_index == 3'd1) ? latched_packet[55:48] :
                             (byte_index == 3'd2) ? latched_packet[47:40] :
                             (byte_index == 3'd3) ? latched_packet[39:32] :
                             (byte_index == 3'd4) ? latched_packet[31:24] :
                             (byte_index == 3'd5) ? latched_packet[23:16] :
                             (byte_index == 3'd6) ? latched_packet[15:8]  :
                                                    latched_packet[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            mcu_wake_irq   <= 1'b0;
            spi_busy       <= 1'b0;
            ip_tx_en       <= 1'b0;
            ip_waddr       <= 3'd0;
            ip_wdata       <= 8'd0;
            us_timer       <= 16'd0;
            byte_index     <= 3'd0;
            byte_delay_cnt <= 6'd0;
            latched_packet <= 64'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    ip_tx_en <= 1'b0;

                    // KHI CÓ CỜ VA CHẠM HOẶC CÓ GÓI ĐẶC TRƯNG TÍNH XONG
                    if (wake_up_trigger || feature_valid) begin
                        // 1. KÉO NGẮT GPIO NGAY LẬP TỨC TRONG 1 CHU KỲ CLOCK (20 ns)!
                        mcu_wake_irq   <= 1'b1;
                        spi_busy       <= 1'b1; // Khóa backpressure FSM math_core
                        latched_packet <= {feature_pitch, feature_roll, feature_rms, feature_var};

                        // 2. KHỞI TẠO BỘ ĐẾM TRỄ MICRO-GIÂY ĐỂ MCU KỊP THỨC DẬY
                        us_timer       <= WAKE_DELAY_CYCLES;
                        state          <= ST_WAKE_DELAY;
                    end else begin
                        mcu_wake_irq <= 1'b0;
                        spi_busy     <= 1'b0;
                    end
                end

                ST_WAKE_DELAY: begin
                    // Đếm ngược micro-giây chờ MCU Sonix SN32F407 thoát Deep-Sleep
                    if (us_timer > 16'd0) begin
                        us_timer <= us_timer - 1'b1;
                    end else begin
                        // Đã hết thời gian chờ micro-giây, bắt đầu đẩy byte đầu tiên
                        byte_index <= 3'd0;
                        state      <= ST_LOAD_BYTE;
                    end
                end

                ST_LOAD_BYTE: begin
                    // Nạp 1 byte vào IP Core SPI Master
                    ip_tx_en <= 1'b1;
                    ip_waddr <= 3'd0;
                    ip_wdata <= current_tx_byte;
                    byte_delay_cnt <= 6'd32; // Chờ IP truyền xong 8 bit trên SCK
                    state    <= ST_WAIT_BYTE;
                end

                ST_WAIT_BYTE: begin
                    ip_tx_en <= 1'b0;
                    if (byte_delay_cnt > 6'd0) begin
                        byte_delay_cnt <= byte_delay_cnt - 1'b1;
                    end else begin
                        if (byte_index == 3'd7) begin
                            // Đã truyền xong trọn vẹn 8 byte (64-bit)
                            state <= ST_FINISH;
                        end else begin
                            byte_index <= byte_index + 1'b1;
                            state      <= ST_LOAD_BYTE;
                        end
                    end
                end

                ST_FINISH: begin
                    // Kết thúc gói truyền: hạ cờ bận và giải phóng ngắt
                    mcu_wake_irq <= 1'b0;
                    spi_busy     <= 1'b0;
                    state        <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
