// =============================================================================
// File:        system_top_integration_template.v
// Dự án:       Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Bản quyền:   Nhóm Nghiên Cứu FPGA - ĐHBK
// Phân công:
//   - Minh:   Khối Quản lý Xung nhịp (PLLVR), Giao tiếp MPU6050 (I2C) & Bộ đệm CDC FIFO
//   - Trường: Bộ lọc số FIR 6 kênh (Low-Pass Filter khử rung xe máy)
//   - Luân:   Phân hệ tính toán toán học trung tâm (math_core_top)
//   - Thắng:  Khối SPI Master truyền gói 64-bit sang MCU Sonix SN32F407 & Debug LED
// =============================================================================

`timescale 1ns / 1ps

module system_top (
    // -------------------------------------------------------------------------
    // 1. CHÂN NGOẠI VI VẬT LÝ TRÊN BOARD (PHYSICAL I/O PINS)
    // -------------------------------------------------------------------------
    input  wire        sys_clkin,       // 27 MHz từ thạch anh onboard OneKiwi 4K
    input  wire        ext_rst_n,       // Nút nhấn Reset phần cứng (Active-Low)

    // Giao tiếp I2C với cảm biến IMU MPU6050 (Phân hệ Minh)
    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    // Giao tiếp SPI Master với MCU Sonix SN32F407 (Phân hệ Thắng)
    output wire        spi_sck,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs_n,

    // Đường ngắt phần cứng đánh thức MCU tức thì khi va chạm (Phân hệ Luân)
    output wire        mcu_wake_irq,

    // Đèn LED hiển thị trạng thái hệ thống
    output wire        led_pll_lock,
    output wire        led_heartbeat
);

    // =========================================================================
    // 0. BỘ PHÁT XUNG NHỊP TOÀN CỤC & TẠO RESET HỆ THỐNG (MINH)
    // =========================================================================
    wire clk_50m;
    wire pll_lock;
    assign led_pll_lock = pll_lock;

    Gowin_PLLVR u_pllvr (
        .clkout (clk_50m),
        .lock   (pll_lock),
        .clkin  (sys_clkin)
    );

    // Reset đồng bộ hóa theo miền 50 MHz
    wire sys_rst_n = ext_rst_n & pll_lock;

    // =========================================================================
    // 1. TÍN HIỆU LIÊN KẾT GIỮA CÁC PHÂN HỆ (INTERCONNECT BUS WIRES)
    // =========================================================================

    // Dữ liệu thô từ I2C MPU6050 (Minh) -> Bộ lọc FIR (Trường)
    wire signed [15:0] raw_acc_x, raw_acc_y, raw_acc_z;
    wire signed [15:0] raw_gyro_x, raw_gyro_y, raw_gyro_z;
    wire               raw_data_valid;

    // Dữ liệu đã lọc từ Bộ lọc FIR (Trường) -> FIFO (Minh)
    wire signed [15:0] fir_acc_x, fir_acc_y, fir_acc_z;
    wire signed [15:0] fir_gyro_x, fir_gyro_y, fir_gyro_z;
    wire               fir_data_valid;

    // Giao tiếp đọc FIFO (Minh) <-> math_core (Luân)
    wire signed [15:0] fifo_acc_x, fifo_acc_y, fifo_acc_z;
    wire signed [15:0] fifo_gyro_x, fifo_gyro_y, fifo_gyro_z;
    wire               fifo_empty;
    wire               fifo_valid;
    wire               fifo_rd_en;

    // Giao tiếp math_core (Luân) <-> SPI Master (Thắng)
    wire signed [15:0] feature_pitch;
    wire signed [15:0] feature_roll;
    wire        [15:0] feature_rms;
    wire        [15:0] feature_var;
    wire               feature_valid;
    wire               spi_busy;

    // =========================================================================
    // 2. KHỞI TẠO MODULE CỦA LUÂN: math_core_top (TRUNG TÂM TÍNH TOÁN)
    //    * ĐÃ CHUẨN HÓA: Dùng trực tiếp clk 50MHz và rst_n từ Minh
    // =========================================================================
    math_core_top #(
        .CONF_ACC_THRESHOLD_SQ (34'h0_0640_0000), // (2.5g * 4096)^2
        .CONF_WAKEUP_PERSIST   (3'd3)             // 3 mẫu liên tiếp để xác nhận va chạm
    ) u_math_core (
        .clk               (clk_50m),
        .rst_n             (sys_rst_n),

        // Giao tiếp với bộ đệm FIFO (Minh)
        .fifo_data_acc_x   (fifo_acc_x),
        .fifo_data_acc_y   (fifo_acc_y),
        .fifo_data_acc_z   (fifo_acc_z),
        .fifo_data_gyro_x  (fifo_gyro_x),
        .fifo_data_gyro_y  (fifo_gyro_y),
        .fifo_data_gyro_z  (fifo_gyro_z),
        .fifo_empty        (fifo_empty),
        .fifo_valid        (fifo_valid),
        .fifo_rd_en        (fifo_rd_en),

        // Ngắt phần cứng đánh thức MCU Sonix GPIO (Zero-latency)
        .wake_up_irq       (mcu_wake_irq),

        // Giao tiếp với bộ phát SPI Master (Thắng)
        .feature_pitch     (feature_pitch),
        .feature_roll      (feature_roll),
        .feature_rms       (feature_rms),
        .feature_var       (feature_var),
        .feature_valid     (feature_valid),
        .spi_busy          (spi_busy)
    );

    // =========================================================================
    // 3. CÁC KHỐI CON CỦA CÁC THÀNH VIÊN KHÁC (STUB / PLACEHOLDER CHO TÍCH HỢP)
    // =========================================================================

    /*
    // --- KHỐI CỦA MINH: I2C MASTER ĐỌC MPU6050 (1 kHz) ---
    mpu6050_i2c_master u_mpu6050_reader (
        .clk        (clk_50m),
        .rst_n      (sys_rst_n),
        .i2c_scl    (i2c_scl),
        .i2c_sda    (i2c_sda),
        .acc_x      (raw_acc_x),
        .acc_y      (raw_acc_y),
        .acc_z      (raw_acc_z),
        .gyro_x     (raw_gyro_x),
        .gyro_y     (raw_gyro_y),
        .gyro_z     (raw_gyro_z),
        .data_valid (raw_data_valid)
    );

    // --- KHỐI CỦA TRƯỜNG: BỘ LỌC FIR 6 KÊNH ---
    fir_filter_bank_6ch u_fir_bank (
        .clk        (clk_50m),
        .rst_n      (sys_rst_n),
        .data_in_en (raw_data_valid),
        .in_acc_x   (raw_acc_x),
        .in_acc_y   (raw_acc_y),
        .in_acc_z   (raw_acc_z),
        .in_gyro_x  (raw_gyro_x),
        .in_gyro_y  (raw_gyro_y),
        .in_gyro_z  (raw_gyro_z),
        .out_acc_x  (fir_acc_x),
        .out_acc_y  (fir_acc_y),
        .out_acc_z  (fir_acc_z),
        .out_gyro_x (fir_gyro_x),
        .out_gyro_y (fir_gyro_y),
        .out_gyro_z (fir_gyro_z),
        .data_out_en(fir_data_valid)
    );

    // --- KHỐI CỦA MINH: BỘ ĐỆM FIFO (Ghi từ FIR, Đọc bởi math_core) ---
    cdc_fifo_buffer u_fifo (
        .clk        (clk_50m),
        .rst_n      (sys_rst_n),
        // Cổng ghi (Write Port - từ FIR của Trường)
        .wr_en      (fir_data_valid),
        .wr_acc_x   (fir_acc_x),
        .wr_acc_y   (fir_acc_y),
        .wr_acc_z   (fir_acc_z),
        .wr_gyro_x  (fir_gyro_x),
        .wr_gyro_y  (fir_gyro_y),
        .wr_gyro_z  (fir_gyro_z),
        // Cổng đọc (Read Port - cấp cho math_core của Luân)
        .rd_en      (fifo_rd_en),
        .rd_acc_x   (fifo_acc_x),
        .rd_acc_y   (fifo_acc_y),
        .rd_acc_z   (fifo_acc_z),
        .rd_gyro_x  (fifo_gyro_x),
        .rd_gyro_y  (fifo_gyro_y),
        .rd_gyro_z  (fifo_gyro_z),
        .empty      (fifo_empty),
        .valid      (fifo_valid)
    );

    // --- KHỐI CỦA THẮNG: SPI MASTER PHÁT GÓI ĐẶC TRƯNG 64-BIT SANG MCU ---
    spi_master_tx u_spi_tx (
        .clk           (clk_50m),
        .rst_n         (sys_rst_n),
        .feature_valid (feature_valid),
        .feature_pitch (feature_pitch),
        .feature_roll  (feature_roll),
        .feature_rms   (feature_rms),
        .feature_var   (feature_var),
        .spi_busy      (spi_busy),
        .spi_sck       (spi_sck),
        .spi_mosi      (spi_mosi),
        .spi_miso      (spi_miso),
        .spi_cs_n      (spi_cs_n)
    );
    */

endmodule
