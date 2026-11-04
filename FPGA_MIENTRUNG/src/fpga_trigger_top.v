// =============================================================================
// File:        fpga_trigger_top.v
// Project:     FPGA_MIENTRUNG - Edge AI Motorcycle Accident Detection System
// Target:      Gowin GW1NSR-LV4CQN48PC6/I5 (Kit OneKiwi 4K / Tang Nano 4K)
// Purpose:     1. Nhịp tim Heartbeat LED trên Pin 10 (1 Hz).
//              2. Nút K1 (Pin 14 - Active Low) / K2 (Pin 15):
//                 - Chống dội phím phần cứng (Debounce 20ms).
//                 - Luân chuyển qua 4 bộ đặc trưng va chạm ngẫu nhiên/chuẩn đoán:
//                   * Bộ 1: Pitch = 1250, Roll = 7850, RMS = 980,  Jerk = 400
//                   * Bộ 2: Pitch = 2560, Roll = 8960, RMS = 1280, Jerk = 512
//                   * Bộ 3: Pitch = -1536, Roll = -7680, RMS = 1152, Jerk = 448
//                   * Bộ 4: Pitch = 3840, Roll = 9600, RMS = 1536, Jerk = 640
//                 - Phát xung ngắt WAKE IRQ trên Pin 41 (Dây Đỏ -> MCU P2.9).
//                 - Đẩy trọn vẹn gói 64-bit (4 x 16-bit) qua bus SPI Master:
//                   * Pin 44 (Dây Xanh biển): SCK  -> P1.0 MCU
//                   * Pin 42 (Dây Xanh lá):   MOSI -> P1.2 MCU
//                   * Pin 46 (Dây Trắng):     CS_N -> P1.8 MCU
//                 - Bật sáng LED2 (Pin 13) báo hiệu trạng thái kích hoạt.
// =============================================================================

`timescale 1ns / 1ps

module fpga_trigger_top (
    input  wire sys_clkin,          // 27 MHz onboard crystal (Pin 45)
    input  wire btn_s1,             // Nút K1 / S1 onboard (Pin 14, Active-Low)
    input  wire btn_s2,             // Nút K2 / S2 onboard (Pin 15, Active-Low)

    // Diagnostic LEDs onboard
    output wire led_onboard_1,      // Pin 10: 1 Hz heartbeat LED
    output wire led_onboard_2,      // Pin 13: Trigger Active LED (HIGH = Active)

    // Trigger signals to MCU Sonix SN32F407 (Active-HIGH pulse)
    // Chân Pin 41 (Dây ĐỎ) nối thẳng sang chân P2.9 (Cụm J7 Pin 4) của MCU Sonix
    output wire trigger_p41,        // Pin 41: WAKE IRQ (Dây Đỏ -> P2.9 MCU)
    output wire trigger_p32,        // Pin 32
    output wire trigger_p31,        // Pin 31
    output wire trigger_p27,        // Pin 27
    output wire trigger_p28,        // Pin 28

    // SPI lines to MCU Sonix SN32F407 (Cụm J12)
    output wire spi_sck,            // Pin 44: SCK (Dây Xanh biển -> P1.0 MCU)
    input  wire spi_miso,           // Pin 43: MISO (Dây Tím <- P1.1 MCU)
    output wire spi_mosi,           // Pin 42: MOSI (Dây Xanh lá -> P1.2 MCU)
    output wire spi_cs_n,           // Pin 46: CS_N (Dây Trắng -> P1.8 MCU)

    // I2C lines to MPU6050 (Header J1)
    output wire i2c_scl,            // Pin 39: SCL
    inout  wire i2c_sda             // Pin 40: SDA
);

    assign i2c_scl = 1'b1;          // Idle high
    assign i2c_sda = 1'bz;          // High-Z

    // -------------------------------------------------------------------------
    // 1. NHỊP TIM HỆ THỐNG (27 MHz -> 1 Hz) TRÊN LED ONBOARD PIN 10
    // -------------------------------------------------------------------------
    reg [24:0] hb_cnt = 25'd0;
    reg        hb_led = 1'b0;

    always @(posedge sys_clkin) begin
        if (hb_cnt >= 25'd13_499_999) begin
            hb_cnt <= 25'd0;
            hb_led <= ~hb_led;
        end else begin
            hb_cnt <= hb_cnt + 1'b1;
        end
    end

    assign led_onboard_1 = hb_led;

    // -------------------------------------------------------------------------
    // 2. CHỐNG DỘI PHÍM PHẦN CỨNG NÚT K1 (Pin 14) & K2 (Pin 15)
    // -------------------------------------------------------------------------
    reg [1:0]  s1_sync = 2'b11;
    reg [1:0]  s2_sync = 2'b11;
    reg [19:0] db_cnt  = 20'd0;     // Đếm 540,000 chu kỳ @ 27 MHz = 20 ms
    reg        btn_stable = 1'b0;
    reg        btn_prev   = 1'b0;
    wire       k_raw_pressed = (s1_sync[1] == 1'b0) || (s2_sync[1] == 1'b0);

    always @(posedge sys_clkin) begin
        s1_sync <= {s1_sync[0], btn_s1};
        s2_sync <= {s2_sync[0], btn_s2};

        if (k_raw_pressed) begin
            if (db_cnt < 20'd540_000) begin
                db_cnt <= db_cnt + 1'b1;
            end else begin
                btn_stable <= 1'b1;
            end
        end else begin
            db_cnt <= 20'd0;
            btn_stable <= 1'b0;
        end

        btn_prev <= btn_stable;
    end

    wire btn_edge = (btn_stable && !btn_prev); // Xung kích hoạt duy nhất 1 chu kỳ khi nhấn phím K1

    // -------------------------------------------------------------------------
    // 3. 4 BỘ ĐẶC TRƯNG MÔ PHỎNG VA CHẠM (64-BIT: PITCH, ROLL, RMS, JERK)
    // -------------------------------------------------------------------------
    reg [1:0] dataset_sel = 2'd0;
    reg signed [15:0] feat_pitch;
    reg signed [15:0] feat_roll;
    reg        [15:0] feat_rms;
    reg        [15:0] feat_jerk;

    always @(*) begin
        case (dataset_sel)
            2'd0: begin
                // Bộ 1: Góc nghiêng 30.7°, Gia tốc 3.83g (Va chạm bên hông xe)
                feat_pitch = 16'sd1250;
                feat_roll  = 16'sd7850;
                feat_rms   = 16'd980;
                feat_jerk  = 16'd400;
            end
            2'd1: begin
                // Bộ 2: Góc nghiêng 35.0°, Gia tốc 5.00g (Lật xe nghiêm trọng)
                feat_pitch = 16'sd2560;
                feat_roll  = 16'sd8960;
                feat_rms   = 16'd1280;
                feat_jerk  = 16'd512;
            end
            2'd2: begin
                // Bộ 3: Góc nghiêng -30.0°, Gia tốc 4.50g (Trượt ngã sườn trái)
                feat_pitch = -16'sd1536;
                feat_roll  = -16'sd7680;
                feat_rms   = 16'd1152;
                feat_jerk  = 16'd448;
            end
            2'd3: begin
                // Bộ 4: Góc nghiêng 37.5°, Gia tốc 6.00g (Đâm va trực diện mạnh)
                feat_pitch = 16'sd3840;
                feat_roll  = 16'sd9600;
                feat_rms   = 16'd1536;
                feat_jerk  = 16'd640;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // 4. MÁY TRẠNG THÁI FSM: ĐÁNH THỨC MCU & TRUYỀN 4 ĐẶC TRƯNG QUA SPI
    // -------------------------------------------------------------------------
    localparam ST_IDLE       = 3'd0,
               ST_WAKE_WAIT  = 3'd1,
               ST_CS_LOW     = 3'd2,
               ST_TX_BIT     = 3'd3,
               ST_CS_HIGH    = 3'd4,
               ST_HOLD_WAKE  = 3'd5;

    reg [2:0]  state = ST_IDLE;
    reg [31:0] timer = 32'd0;
    reg [5:0]  bit_idx = 6'd63;
    reg [6:0]  sck_div = 7'd0;      // 100 chu kỳ @ 27 MHz = 270 kHz
    reg [63:0] tx_shift = 64'd0;

    reg r_trigger_p41 = 1'b0;
    reg r_spi_cs_n    = 1'b1;
    reg r_spi_sck     = 1'b0;
    reg r_spi_mosi    = 1'b0;

    // Khóa khởi động 500ms chống sốc nguồn
    reg [23:0] power_on_lock = 24'd13_500_000;

    always @(posedge sys_clkin) begin
        if (power_on_lock > 24'd0) begin
            power_on_lock <= power_on_lock - 1'b1;
            state         <= ST_IDLE;
            r_trigger_p41 <= 1'b0;
            r_spi_cs_n    <= 1'b1;
            r_spi_sck     <= 1'b0;
            r_spi_mosi    <= 1'b0;
            dataset_sel   <= 2'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    r_trigger_p41 <= 1'b0;
                    r_spi_cs_n    <= 1'b1;
                    r_spi_sck     <= 1'b0;
                    r_spi_mosi    <= 1'b0;

                    // Khi người dùng bấm nút K1 (btn_s1)
                    if (btn_edge) begin
                        // 1. Kéo chân ngắt P41 lên mức HIGH tức thời để đánh thức MCU!
                        r_trigger_p41 <= 1'b1;

                        // 2. Nạp gói 64-bit đặc trưng hiện tại
                        tx_shift <= {feat_pitch, feat_roll, feat_rms, feat_jerk};

                        // 3. Tăng bộ đếm để lần nhấn sau sẽ gửi bộ số tiếp theo
                        dataset_sel <= dataset_sel + 1'b1;

                        // 4. Chờ 1.0 ms (27,000 chu kỳ) để MCU thức dậy và sẵn sàng nhận SPI
                        timer <= 32'd27_000;
                        state <= ST_WAKE_WAIT;
                    end
                end

                ST_WAKE_WAIT: begin
                    if (timer > 32'd0) begin
                        timer <= timer - 1'b1;
                    end else begin
                        // Bắt đầu phiên SPI: Hạ CS_N xuống LOW
                        r_spi_cs_n <= 1'b0;
                        timer      <= 32'd270; // Giữ CS LOW 10 us trước xung clock đầu tiên
                        state      <= ST_CS_LOW;
                    end
                end

                ST_CS_LOW: begin
                    if (timer > 32'd0) begin
                        timer <= timer - 1'b1;
                    end else begin
                        bit_idx <= 6'd63; // Bắt đầu phát từ bit cao nhất (MSB)
                        sck_div <= 7'd0;
                        state   <= ST_TX_BIT;
                    end
                end

                ST_TX_BIT: begin
                    // Phát SCK 270 kHz (100 chu kỳ / bit: 50 chu kỳ mức thấp, 50 chu kỳ mức cao)
                    if (sck_div == 7'd0) begin
                        r_spi_mosi <= tx_shift[bit_idx]; // Đặt dữ liệu khi SCK còn LOW
                        r_spi_sck  <= 1'b0;
                        sck_div    <= sck_div + 1'b1;
                    end else if (sck_div == 7'd50) begin
                        r_spi_sck  <= 1'b1;              // Cạnh lên SCK: MCU chốt dữ liệu
                        sck_div    <= sck_div + 1'b1;
                    end else if (sck_div >= 7'd99) begin
                        r_spi_sck  <= 1'b0;              // Cạnh xuống SCK: Kết thúc 1 bit
                        sck_div    <= 7'd0;
                        if (bit_idx == 6'd0) begin
                            // Đã phát xong trọn vẹn 64 bits (8 bytes = 4 đặc trưng)
                            timer <= 32'd270;           // Hold time 10 us
                            state <= ST_CS_HIGH;
                        end else begin
                            bit_idx <= bit_idx - 1'b1;
                        end
                    end else begin
                        sck_div <= sck_div + 1'b1;
                    end
                end

                ST_CS_HIGH: begin
                    // Đóng phiên SPI: Kéo CS_N lên HIGH
                    r_spi_cs_n <= 1'b1;
                    r_spi_sck  <= 1'b0;
                    r_spi_mosi <= 1'b0;

                    if (timer > 32'd0) begin
                        timer <= timer - 1'b1;
                    end else begin
                        // Tiếp tục giữ chân WAKE IRQ Pin 41 mức HIGH thêm 300 ms
                        // để đèn LED sáng rõ và MCU hoàn thành xử lý
                        timer <= 32'd8_100_000; // 300 ms
                        state <= ST_HOLD_WAKE;
                    end
                end

                ST_HOLD_WAKE: begin
                    if (timer > 32'd0) begin
                        timer <= timer - 1'b1;
                    end else begin
                        // Hạ ngắt WAKE IRQ về 0 và quay lại trạng thái IDLE
                        r_trigger_p41 <= 1'b0;
                        state         <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // Gán tín hiệu ngõ ra phần cứng
    assign trigger_p41    = r_trigger_p41;
    assign trigger_p32    = r_trigger_p41;
    assign trigger_p31    = r_trigger_p41;
    assign trigger_p27    = r_trigger_p41;
    assign trigger_p28    = r_trigger_p41;

    assign spi_sck        = r_spi_sck;
    assign spi_mosi       = r_spi_mosi;
    assign spi_cs_n       = r_spi_cs_n;

    // LED2 sáng lên bất cứ khi nào đang trong phiên kích hoạt
    assign led_onboard_2  = r_trigger_p41;

endmodule
