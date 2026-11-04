// =============================================================================
// File:        mpu6050_uart_test_top.v
// Dự án:       Kiểm thử cảm biến MPU6050 & FPGA qua giao tiếp UART
// Nền tảng:    Gowin GW1NSR-LV4CQN48PC6/I5 (OneKiwi 4K / Tang Nano 4K)
// Mô tả:        Top module kiểm thử thực tế cảm biến MPU6050 và xuất dữ liệu qua UART:
//               - Tự động giao tiếp I2C đọc MPU6050 ở tần số quét 50 Hz.
//               - Chuyển đổi dữ liệu gia tốc sang ký tự ASCII dễ đọc.
//               - Xuất dữ liệu qua chân UART TX (Pin 46) ở tốc độ 115200 bps.
//               - 3 đèn LED báo trực quan: Nhịp tim hệ thống (Pin 32),
//                 Phát hiện MPU6050 (Pin 23), Xung nhận mẫu (Pin 18).
// =============================================================================

`timescale 1ns / 1ps

module mpu6050_uart_test_top (
    input  wire        sys_clkin,        // Xung nhịp 27 MHz từ thạch anh onboard (Pin 45)

    // Giao tiếp I2C với cảm biến MPU6050 (3.3V)
    inout  wire        i2c_scl,          // Chân SCL (Pin 44 - 3-44)
    inout  wire        i2c_sda,          // Chân SDA (Pin 39 - 3-39)

    // Ngõ ra UART TX gửi lên máy tính qua mạch chuyển USB-TTL (Pin 31 - 2-31)
    output wire        uart_tx_pin,      // Chân UART TX (Pin 31 - 2-31)

    // Đèn LED hiển thị trực quan trên board
    output reg         led_heartbeat,    // LED nhấp nháy ~1 Hz báo hệ thống đang chạy (Pin 32)
    output wire        led_i2c_ok,       // LED sáng khi MPU6050 phản hồi I2C thành công (Pin 23)
    output reg         led_data_rx       // LED chớp tắt mỗi khi nhận được 1 mẫu mới (Pin 18)
);


    // -------------------------------------------------------------------------
    // 1. TẠO XUNG CLOCK 50 MHz VÀ ĐỒNG BỘ RESET
    // -------------------------------------------------------------------------
    wire clk_50mhz;

    Clock_manager u_clock_mgr (
        .clk        (sys_clkin),
        .rst_n      (1'b1),
        .clk_50MHz  (clk_50mhz)
    );

    // Mạch tự động tạo xung Power-On Reset nội bộ:
    // Tự động giải phóng Reset sau 5ms (250,000 chu kỳ 50 MHz),
    // hoàn toàn không phụ thuộc vào trạng thái trôi nổi của chân ngoài.
    reg [20:0] por_cnt = 21'd0;
    always @(posedge clk_50mhz) begin
        if (por_cnt < 21'd250_000)
            por_cnt <= por_cnt + 1'b1;
    end
    wire sys_rst_n = (por_cnt >= 21'd250_000);

    // -------------------------------------------------------------------------
    // 2. KHỐI ĐỌC CẢM BIẾN MPU6050 QUA I2C PHẦN CỨNG
    // -------------------------------------------------------------------------
    wire signed [15:0] acc_x;
    wire signed [15:0] acc_y;
    wire signed [15:0] acc_z;
    wire [7:0]         who_am_i;
    wire               data_valid;
    wire               i2c_device_ok;
    wire               sda_stuck_low;

    mpu6050_hardware_reader u_mpu6050 (
        .clk           (clk_50mhz),
        .rst_n         (sys_rst_n),
        .i2c_scl       (i2c_scl),
        .i2c_sda       (i2c_sda),
        .acc_x         (acc_x),
        .acc_y         (acc_y),
        .acc_z         (acc_z),
        .who_am_i      (who_am_i),
        .data_valid    (data_valid),
        .i2c_device_ok (i2c_device_ok),
        .sda_stuck_low (sda_stuck_low)
    );

    // -------------------------------------------------------------------------
    // 3. KHỐI ĐỔI DỮ LIỆU SANG ASCII & PHÁT UART
    // -------------------------------------------------------------------------
    wire       tx_start;
    wire [7:0] tx_data;
    wire       tx_busy;

    sensor_to_ascii u_formatter (
        .clk           (clk_50mhz),
        .rst_n         (sys_rst_n),
        .data_valid    (data_valid),
        .acc_x         (acc_x),
        .acc_y         (acc_y),
        .acc_z         (acc_z),
        .device_ok     (i2c_device_ok),
        .who_am_i      (who_am_i),
        .sda_stuck_low (sda_stuck_low),
        .tx_start      (tx_start),
        .tx_data       (tx_data),
        .tx_busy       (tx_busy)
    );

    uart_tx #(
        .CLK_FREQ  (50_000_000),
        .BAUD_RATE (115200)
    ) u_uart (
        .clk      (clk_50mhz),
        .rst_n    (sys_rst_n),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx_pin   (uart_tx_pin),
        .tx_busy  (tx_busy)
    );

    // -------------------------------------------------------------------------
    // 4. ĐÈN LED CHỈ THỊ TRẠNG THÁI
    // -------------------------------------------------------------------------
    // LED 1 (Heartbeat): Đảo trạng thái mỗi 0.5s
    reg [24:0] hb_cnt;
    always @(posedge clk_50mhz or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            hb_cnt        <= 25'd0;
            led_heartbeat <= 1'b0;
        end else begin
            if (hb_cnt >= 25'd24_999_999) begin
                hb_cnt        <= 25'd0;
                led_heartbeat <= ~led_heartbeat;
            end else begin
                hb_cnt <= hb_cnt + 1'b1;
            end
        end
    end

    // LED 2 (I2C OK): Sáng khi MPU6050 gửi phản hồi ACK thành công
    // Lưu ý: LED trên board OneKiwi 4K là Active-Low (Mức 0 sáng, mức 1 tắt) hoặc Active-High tùy Bank
    assign led_i2c_ok = ~i2c_device_ok; // Kéo mức 0 để LED sáng khi OK

    // LED 3 (Data Valid): Chớp tắt mỗi khi nhận mẫu hợp lệ
    always @(posedge clk_50mhz or negedge sys_rst_n) begin
        if (!sys_rst_n)
            led_data_rx <= 1'b1;
        else if (data_valid)
            led_data_rx <= ~led_data_rx;
    end

endmodule
