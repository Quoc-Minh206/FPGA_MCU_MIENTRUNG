// =============================================================================
// File:        top_module.v
// Dự án:       Hệ thống Edge AI Phát Hiện Tai Nạn & Thu Thập Dữ Liệu Xe Máy
// Nền tảng:    Gowin GW1NSR-LV4CQN48PC6/I5 (Kit OneKiwi 4K / Tang Nano 4K)
// Mô tả:       Tích hợp toàn diện:
//              1. Đọc cảm biến MPU6050 (GY-521) qua I2C phần cứng (SCL=39, SDA=40)
//              2. Ghi dữ liệu mẫu liên tục vào thẻ nhớ MicroSD qua SPI (CS=46, SCK=43, MOSI=42, MISO=41)
//              3. Truyền dữ liệu trực tiếp lên máy tính qua UART TX (Pin 31, 115200 baud)
//              4. Xử lý số học thời gian thực: Lọc số FIR -> FIFO -> Math Core CORDIC
//              5. Đèn LED hiển thị trực quan trạng thái hệ thống
// =============================================================================

`timescale 1ns / 1ps

module top_module (
    input  wire        sys_clkin,        // Xung nhịp 27 MHz onboard (Pin 45)
    input  wire        ext_rst_n,        // Nút nhấn Reset phần cứng (Pin 14: KEY1)

    // Bus I2C nối cảm biến MPU6050 GY-521 (Bank 1 - 3.3V)
    inout  wire        i2c_scl,          // Pin 39 (SCL)
    inout  wire        i2c_sda,          // Pin 40 (SDA)

    // Kênh UART Debug Monitor trực tiếp lên máy tính / PuTTY qua cổng micro-USB (Bank 2 - 3.3V)
    output wire        uart_tx_pin,      // Pin 31 (UART TX, 115200 baud)
    output wire        uart_tx_p44,      // Pin 44 (UART TX dự phòng Bank 1 3.3V)
    output wire        uart_tx_p27,      // Pin 27 (UART TX dự phòng Bank 2 Header J1)
    output wire        uart_tx_p28,      // Pin 28 (UART TX dự phòng Bank 2 Header J1)

    // Giao tiếp SPI với MicroSD Card Adapter (Bank 1 & 2 - 3.3V)
    output wire        sd_cs,            // Pin 46 (CS)
    output wire        sd_sck,           // Pin 43 (SCK)
    output wire        sd_mosi,          // Pin 42 (MOSI)
    input  wire        sd_miso,          // Pin 41 (MISO)

    // Đèn LED hiển thị trạng thái hệ thống:
    // 1. Đèn LED onboard trực tiếp trên bo mạch Kiwi 4K / Tang Nano 4K:
    output wire        led_onboard_1,    // Pin 10: LED1 onboard nhấp nháy nhịp tim 1 Hz
    output wire        led_onboard_2,    // Pin 13: LED2 onboard chớp tắt khi ghi thẻ MicroSD
    // 2. Chân LED mở rộng ra hàng chân Header:
    output wire        led_heartbeat,    // Pin 32 (Đồng thời phát UART TX bảo hiểm)
    output wire        led_imu_ok,       // Pin 23 (Sáng khi MPU6050 nhận diện ID 0x68 thành công)
    output wire        led_rec_active    // Pin 18 (Báo nhấp nháy khi ghi mẫu / thẻ nhớ MicroSD)
);

    // =========================================================================
    // 0. ĐỒNG BỘ TOÀN HỆ THỐNG VÀ POWER-ON RESET (POR) NỘI BỘ 27 MHz
    //    Tự động giải phóng Reset sau 10ms, đồng bộ hóa an toàn với nút nhấn ngoài KEY1
    // =========================================================================
    reg [21:0] por_cnt        = 22'd0;
    reg        sys_rst_n      = 1'b0;
    reg        ext_rst_sync_0 = 1'b1;
    reg        ext_rst_sync_1 = 1'b1;

    always @(posedge sys_clkin) begin
        ext_rst_sync_0 <= ext_rst_n;
        ext_rst_sync_1 <= ext_rst_sync_0;

        if (por_cnt < 22'd270_000) begin
            por_cnt   <= por_cnt + 1'b1;
            sys_rst_n <= 1'b0;
        end else begin
            sys_rst_n <= ext_rst_sync_1;
        end
    end

    // =========================================================================
    // 1. KHỐI ĐỌC CẢM BIẾN MPU6050 QUA I2C PHẦN CỨNG (SCL=39, SDA=40)
    // =========================================================================
    wire signed [15:0] raw_ax, raw_ay, raw_az;
    wire [7:0]         who_am_i;
    wire               data_valid, device_ok, sda_stuck_low;

    mpu6050_hardware_reader u_reader (
        .clk           (sys_clkin),     // 27 MHz thạch anh trực tiếp, không phụ thuộc PLL
        .rst_n         (sys_rst_n),
        .i2c_scl       (i2c_scl),       // Chân 39 (SCL)
        .i2c_sda       (i2c_sda),       // Chân 40 (SDA)
        .acc_x         (raw_ax),
        .acc_y         (raw_ay),
        .acc_z         (raw_az),
        .who_am_i      (who_am_i),
        .data_valid    (data_valid),
        .i2c_device_ok (device_ok),
        .sda_stuck_low (sda_stuck_low)
    );

    // =========================================================================
    // 2. KHỐI THU THẬP VÀ GHI DỮ LIỆU VÀO THẺ NHỚ MICROSD (SPI MODE)
    // =========================================================================
    wire        sd_wr_busy;
    wire        sd_init_done;
    wire [31:0] sd_sector_addr;

    sd_card_logger u_sd_logger (
        .clk            (sys_clkin),     // 27 MHz
        .rst_n          (sys_rst_n),
        .data_valid     (data_valid),
        .acc_x          (raw_ax),
        .acc_y          (raw_ay),
        .acc_z          (raw_az),
        .sd_cs          (sd_cs),         // Pin 46
        .sd_sck         (sd_sck),        // Pin 43
        .sd_mosi        (sd_mosi),       // Pin 42
        .sd_miso        (sd_miso),       // Pin 41
        .wr_busy        (sd_wr_busy),
        .init_done      (sd_init_done),
        .sector_address (sd_sector_addr)
    );

    // =========================================================================
    // 3. KÊNH UART DEBUG MONITOR GỬI LÊN LAPTOP (115200 BAUD)
    // =========================================================================
    wire       uart_tx_busy, uart_tx_start, uart_tx_signal;
    wire [7:0] uart_tx_data;

    sensor_to_ascii u_ascii (
        .clk           (sys_clkin),     // 27 MHz
        .rst_n         (sys_rst_n),
        .data_valid    (data_valid),
        .acc_x         (raw_ax),
        .acc_y         (raw_ay),
        .acc_z         (raw_az),
        .device_ok     (device_ok),
        .who_am_i      (who_am_i),
        .sda_stuck_low (sda_stuck_low),
        .tx_start      (uart_tx_start),
        .tx_data       (uart_tx_data),
        .tx_busy       (uart_tx_busy)
    );

    uart_tx #(
        .CLK_FREQ  (27_000_000),        // 27 MHz chuẩn thạch anh
        .BAUD_RATE (115200)
    ) u_uart (
        .clk      (sys_clkin),
        .rst_n    (sys_rst_n),
        .tx_start (uart_tx_start),
        .tx_data  (uart_tx_data),
        .tx_pin   (uart_tx_signal),
        .tx_busy  (uart_tx_busy)
    );

    // Phát song song UART ra tất cả các chân tiềm năng trên cả 2 hàng Header J1 và J2:
    assign uart_tx_pin   = uart_tx_signal; // Pin 31 (Header J1, Bank 2)
    assign uart_tx_p44   = uart_tx_signal; // Pin 44 (Header J2, Bank 1)
    assign uart_tx_p27   = uart_tx_signal; // Pin 27 (Header J1, Bank 2)
    assign uart_tx_p28   = uart_tx_signal; // Pin 28 (Header J1, Bank 2)
    assign led_heartbeat = uart_tx_signal; // Pin 32 (Header J1, Bank 2)

    // =========================================================================
    // 4. PHÂN HỆ XỬ LÝ TOÁN HỌC & AI THỜI GIAN THỰC (EDGE AI PIPELINE)
    // =========================================================================
    // Chuẩn hóa độ nhạy: dải ±2g (16384 LSB/g) -> Q4.12 (4096 LSB/g, 1.0g = 4096)
    wire signed [15:0] ax_q12 = raw_ax >>> 2;
    wire signed [15:0] ay_q12 = raw_ay >>> 2;
    wire signed [15:0] az_q12 = raw_az >>> 2;

    // Bộ lọc số thông thấp FIR khử rung xe máy (Trường)
    wire signed [15:0] fir_ax;
    wire               fir_done;

    fir_top_module u_fir (
        .clk_50mhz   (sys_clkin),
        .rst_n       (sys_rst_n),
        .data_valid  (data_valid),
        .raw_ax      (ax_q12),
        .filtered_ax (fir_ax),
        .filter_done (fir_done)
    );

    // Bộ đệm FIFO CDC (Minh)
    wire signed [15:0] fifo_ax;
    wire               fifo_empty, fifo_full, fifo_rd_en;
    reg                fifo_valid;

    FIFO_HS_Top u_fifo (
        .Data         (fir_ax),
        .WrClk        (sys_clkin),
        .RdClk        (sys_clkin),
        .WrEn         (fir_done),
        .RdEn         (fifo_rd_en),
        .Almost_Empty (),
        .Almost_Full  (),
        .Q            (fifo_ax),
        .Empty        (fifo_empty),
        .Full         (fifo_full)
    );

    always @(posedge sys_clkin or negedge sys_rst_n)
        fifo_valid <= !sys_rst_n ? 1'b0 : (fifo_rd_en & !fifo_empty);

    // Khối xử lý toán học trung tâm CORDIC & trích xuất đặc trưng (Luân)
    wire signed [15:0] pitch, roll;
    wire        [15:0] rms, var_feat;
    wire               feat_valid, wake_irq, spi_busy;

    math_core_top #(
        .CONF_ACC_THRESHOLD_SQ (34'h0_0640_0000), // (2.5g * 4096)^2
        .CONF_WAKEUP_PERSIST   (3'd3)
    ) u_math (
        .clk              (sys_clkin),
        .rst_n            (sys_rst_n),
        .fifo_data_acc_x  (fifo_ax),
        .fifo_data_acc_y  (ay_q12),
        .fifo_data_acc_z  (az_q12),
        .fifo_data_gyro_x (16'sd0),
        .fifo_data_gyro_y (16'sd0),
        .fifo_data_gyro_z (16'sd0),
        .fifo_empty       (fifo_empty),
        .fifo_valid       (fifo_valid),
        .fifo_rd_en       (fifo_rd_en),
        .wake_up_irq      (wake_irq),
        .feature_pitch    (pitch),
        .feature_roll     (roll),
        .feature_rms      (rms),
        .feature_var      (var_feat),
        .feature_valid    (feat_valid),
        .spi_busy         (spi_busy)
    );

    // Bộ điều khiển SPI Master MCU (Thắng - duy trì logic nội bộ)
    wire internal_mcu_wake, internal_mcu_sck, internal_mcu_mosi, internal_mcu_cs_n;
    sonix_spi_wake_controller #(
        .WAKE_DELAY_CYCLES (16'd250)
    ) u_spi (
        .clk             (sys_clkin),
        .rst_n           (sys_rst_n),
        .wake_up_trigger (wake_irq),
        .feature_valid   (feat_valid),
        .feature_pitch   (pitch),
        .feature_roll    (roll),
        .feature_rms     (rms),
        .feature_var     (var_feat),
        .spi_busy        (spi_busy),
        .mcu_wake_irq    (internal_mcu_wake),
        .spi_sck         (internal_mcu_sck),
        .spi_mosi        (internal_mcu_mosi),
        .spi_miso        (1'b0),
        .spi_cs_n        (internal_mcu_cs_n)
    );

    // =========================================================================
    // 5. ĐÈN LED CHỈ THỊ TRẠNG THÁI TRỰC QUAN
    // =========================================================================
    // 5. ĐÈN LED CHẨN ĐOÁN LỖI PHẦN CỨNG THÔNG MINH
    // =========================================================================
    // Bộ đếm tạo xung chẩn đoán từ thạch anh 27 MHz:
    reg [24:0] slow_cnt   = 25'd0;
    reg [21:0] fast_cnt   = 22'd0;
    reg        slow_blink = 1'b0; // 1 Hz (Chớp chậm 1s/lần)
    reg        fast_blink = 1'b0; // 5 Hz (Chớp nhanh 5 lần/giây)

    always @(posedge sys_clkin) begin
        // Bộ tạo xung 1 Hz (chu kỳ 0.5s đảo trạng thái)
        if (slow_cnt >= 25'd13_499_999) begin
            slow_cnt   <= 25'd0;
            slow_blink <= ~slow_blink;
        end else begin
            slow_cnt <= slow_cnt + 1'b1;
        end

        // Bộ tạo xung 5 Hz (chu kỳ 0.1s đảo trạng thái)
        if (fast_cnt >= 22'd2_699_999) begin
            fast_cnt   <= 22'd0;
            fast_blink <= ~fast_blink;
        end else begin
            fast_cnt <= fast_cnt + 1'b1;
        end
    end

    // [CHẨN ĐOÁN 1]: ĐÈN LED1 ONBOARD (PIN 10) KIỂM TRA RIÊNG CẢM BIẾN MPU-6050
    // - Chớp chậm 1 Hz (1s/lần)   => MPU6050 KẾT NỐI TỐT, đã nhận diện ID 0x68
    // - Chớp nhanh 5 Hz (cảnh báo) => MPU6050 LỖI / CHƯA KẾT NỐI / ĐỨT DÂY SCL-SDA / SAI ĐỊA CHỈ
    assign led_onboard_1 = (device_ok) ? slow_blink : fast_blink;

    // [CHẨN ĐOÁN 2]: ĐÈN LED2 ONBOARD (PIN 13) KIỂM TRA RIÊNG THẺ NHỚ MICROSD
    // - TẮT HOÀN TOÀN             => THẺ SD LỖI / CHƯA CẮM / LỎNG DÂY SPI
    // - SÁNG ĐỨNG                 => THẺ SD ĐÃ SẴN SÀNG, ĐANG CHỜ MẪU TỪ MPU6050
    // - CHỚP TẮT ĐỀU ĐẶN          => ĐANG TÍCH CỰC GHI DỮ LIỆU CÁC SECTOR VÀO THẺ NHỚ
    assign led_onboard_2 = (~sd_init_done) ? 1'b1 : 
                           (sd_sector_addr > 32'd1000) ? sd_sector_addr[1] : 1'b0;

    // Chân LED mở rộng ra Header ngoài
    assign led_imu_ok     = ~device_ok;
    assign led_rec_active = led_onboard_2;

endmodule
