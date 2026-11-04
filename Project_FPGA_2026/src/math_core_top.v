// =============================================================================
// Tên Module:   math_core_top
// Dự án:        Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Phân hệ:      FPGA Pre-processing & Feature Extraction Engine
// Kỹ sư:        Vương Thành Luân
// Nền tảng:     Gowin LittleBee GW1NSR-LV4CQN48PC6/I5 (OneKiwi 4K)
// Vi điều khiển:Sonix SN32F407 (ARM Cortex-M4)
// Mô tả:        Top-level tích hợp toàn bộ phân hệ math_core, kết nối 4 khối con:
//               1. accel_threshold_detector : Phát hiện ngưỡng va chạm & đánh thức MCU tức thời.
//               2. cordic_attitude_estimator: Ước lượng góc nghiêng Roll & Pitch (CORDIC 32 tầng).
//               3. stat_feature_extractor   : Trích xuất đặc trưng RMS^2 và Variance cửa sổ N=32.
//               4. math_pipeline_ctrl       : Điều khiển đồng bộ pipeline, bắt tay FIFO & SPI.
// =============================================================================

`timescale 1ns / 1ps

module math_core_top #(
    // -------------------------------------------------------------------------
    // CÁC THAM SỐ CẤU HÌNH HỆ THỐNG (SYSTEM CONFIGURATION PARAMETERS)
    // -------------------------------------------------------------------------
    // Ngưỡng năng lượng va chạm: 2.5g bình phương = (2.5 * 4096)^2 = 104,857,600 (34'h0_0640_0000)
    parameter [33:0] CONF_ACC_THRESHOLD_SQ = 34'h0_0640_0000,
    // Số mẫu liên tiếp vượt ngưỡng để khẳng định va chạm (chống nhiễu nhọn)
    parameter [2:0]  CONF_WAKEUP_PERSIST   = 3'd3
)(
    // -------------------------------------------------------------------------
    // 1. XUNG NHỊP VÀ RESET HỆ THỐNG (CLOCK & RESET INTERFACE - CHUẨN ĐẶC TẢ)
    // -------------------------------------------------------------------------
    input  wire        clk,               // Xung nhịp hệ thống 50 MHz từ khối Clock Manager (PLLVR của Minh / Top)
    input  wire        rst_n,             // Reset tích cực mức thấp (Active-Low) toàn hệ thống

    // -------------------------------------------------------------------------
    // 2. GIAO TIẾP VỚI BỘ ĐỆM CDC FIFO BUFFER (KẾT NỐI VỚI PHÂN HỆ CỦA MINH)
    // -------------------------------------------------------------------------
    input  wire signed [15:0] fifo_data_acc_x, // Gia tốc trục X đã lọc FIR từ FIFO (Signed Q4.12, dải [-8g, +8g])
    input  wire signed [15:0] fifo_data_acc_y, // Gia tốc trục Y đã lọc FIR từ FIFO (Signed Q4.12, dải [-8g, +8g])
    input  wire signed [15:0] fifo_data_acc_z, // Gia tốc trục Z đã lọc FIR từ FIFO (Signed Q4.12, dải [-8g, +8g])
    input  wire signed [15:0] fifo_data_gyro_x,// Vận tốc góc trục X (Signed Q12.4 dps) - Reserved cho Sensor Fusion
    input  wire signed [15:0] fifo_data_gyro_y,// Vận tốc góc trục Y (Signed Q12.4 dps) - Reserved cho Sensor Fusion
    input  wire signed [15:0] fifo_data_gyro_z,// Vận tốc góc trục Z (Signed Q12.4 dps) - Reserved cho Sensor Fusion
    input  wire        fifo_empty,        // Cờ báo FIFO rỗng (Active-High). math_core chỉ đọc khi fifo_empty == 0
    input  wire        fifo_valid,        // Cờ báo dữ liệu từ FIFO đang hợp lệ (trễ 1 cycle sau fifo_rd_en)
    output wire        fifo_rd_en,        // Lệnh yêu cầu đọc mẫu tiếp theo từ FIFO

    // -------------------------------------------------------------------------
    // 3. TÍN HIỆU NGẮT PHẦN CỨNG ĐÁNH THỨC MCU (WAKE-UP INTERRUPT TO MCU GPIO)
    // -------------------------------------------------------------------------
    output wire        wake_up_irq,       // Xung tích cực HIGH đánh thức MCU Sonix khỏi Deep-Sleep (độ trễ <= 2 chu kỳ)

    // -------------------------------------------------------------------------
    // 4. GIAO TIẾP VỚI KHỐI SPI MASTER (KẾT NỐI VỚI PHÂN HỆ CỦA THẮNG)
    // -------------------------------------------------------------------------
    output wire signed [15:0] feature_pitch, // Đặc trưng góc Pitch ước lượng (Signed Q8.8, dải [-90.00, +90.00] độ)
    output wire signed [15:0] feature_roll,  // Đặc trưng góc Roll ước lượng (Signed Q8.8, dải [-180.00, +180.00] độ)
    output wire        [15:0] feature_rms,   // Đặc trưng năng lượng hiệu dụng RMS^2 (Unsigned Q8.8)
    output wire        [15:0] feature_var,   // Đặc trưng phương sai dao động (Unsigned Q12.4)
    output wire        feature_valid,     // Xung 1 chu kỳ báo gói 4 đặc trưng sẵn sàng nạp vào thanh ghi dịch SPI
    input  wire        spi_busy           // Cờ báo khối SPI đang bận đẩy gói tin cũ sang MCU (Active-High)
);

    // Tham chiếu các ngõ vào Gyro reserved để triệt tiêu cảnh báo CV0016 của Gowin Synthesis
    wire _unused_gyro_ok = &{1'b0, fifo_data_gyro_x, fifo_data_gyro_y, fifo_data_gyro_z};

    // =========================================================================
    // KHAI BÁO CÁC ĐƯỜNG BUS LIÊN KẾT NỘI BỘ (INTERNAL INTERCONNECT SIGNALS)
    // =========================================================================

    // Tín hiệu điều khiển phân phối từ math_pipeline_ctrl tới các sub-modules
    wire        ctrl_ce;                  // Clock-Enable toàn cục cho các khối tính toán
    wire        pipe_sample_valid;        // Xung báo mẫu dữ liệu mới hợp lệ
    wire        pipe_window_last;         // Xung báo mẫu thứ 32 (mẫu cuối cửa sổ khối N=32)
    wire signed [15:0] pipe_acc_x;        // Dữ liệu gia tốc trục X đã chốt an toàn
    wire signed [15:0] pipe_acc_y;        // Dữ liệu gia tốc trục Y đã chốt an toàn
    wire signed [15:0] pipe_acc_z;        // Dữ liệu gia tốc trục Z đã chốt an toàn

    // Tín hiệu chia sẻ từ accel_threshold_detector
    wire [33:0] wake_mag_sq;              // Tổng bình phương Ax^2 + Ay^2 + Az^2 (chia sẻ cho stat_feature_extractor)
    wire        wake_irq_raw;             // Cờ vượt ngưỡng thô (trước persistency)

    // Tín hiệu kết quả từ cordic_attitude_estimator đưa về math_pipeline_ctrl
    wire signed [15:0] cordic_roll_out;   // Góc Roll ước lượng (Signed Q8.8)
    wire signed [15:0] cordic_pitch_out;  // Góc Pitch ước lượng (Signed Q8.8)
    wire        cordic_done;              // Cờ báo CORDIC đã hoàn tất (sau 32 chu kỳ)

    // Tín hiệu kết quả từ stat_feature_extractor đưa về math_pipeline_ctrl
    wire [15:0] stat_rms_out;             // Năng lượng hiệu dụng RMS^2 (Unsigned Q8.8)
    wire [15:0] stat_var_out;             // Phương sai dao động (Unsigned Q12.4)
    wire        stat_done;                // Cờ báo khối Thống kê đã hoàn tất (sau 4 chu kỳ post-window)

    // =========================================================================
    // 1. KHỞI TẠO SUB-MODULE: math_pipeline_ctrl (BỘ ĐIỀU KHIỂN PIPELINE TRUNG TÂM)
    // =========================================================================
    math_pipeline_ctrl u_pipeline_ctrl (
        .clk               (clk),
        .rst_n             (rst_n),

        // Giao tiếp FIFO
        .fifo_empty        (fifo_empty),
        .fifo_valid        (fifo_valid),
        .fifo_rd_en        (fifo_rd_en),
        .fifo_data_acc_x   (fifo_data_acc_x),
        .fifo_data_acc_y   (fifo_data_acc_y),
        .fifo_data_acc_z   (fifo_data_acc_z),

        // Giao tiếp SPI Master
        .spi_busy          (spi_busy),
        .feature_valid     (feature_valid),
        .feature_pitch     (feature_pitch),
        .feature_roll      (feature_roll),
        .feature_rms       (feature_rms),
        .feature_var       (feature_var),

        // Bus phân phối nội bộ
        .ctrl_ce           (ctrl_ce),
        .pipe_sample_valid (pipe_sample_valid),
        .pipe_window_last  (pipe_window_last),
        .pipe_acc_x        (pipe_acc_x),
        .pipe_acc_y        (pipe_acc_y),
        .pipe_acc_z        (pipe_acc_z),

        // Nhận kết quả từ CORDIC
        .cordic_done       (cordic_done),
        .cordic_roll_out   (cordic_roll_out),
        .cordic_pitch_out  (cordic_pitch_out),

        // Nhận kết quả từ Thống kê
        .stat_done         (stat_done),
        .stat_rms_out      (stat_rms_out),
        .stat_var_out      (stat_var_out)
    );

    // =========================================================================
    // 2. KHỞI TẠO SUB-MODULE: accel_threshold_detector (PHÁT HIỆN NGƯỠNG VA CHẠM)
    // =========================================================================
    accel_threshold_detector #(
        .CONF_ACC_THRESHOLD_SQ (CONF_ACC_THRESHOLD_SQ),
        .CONF_WAKEUP_PERSIST   (CONF_WAKEUP_PERSIST)
    ) u_threshold_detector (
        .clk               (clk),
        .rst_n             (rst_n),
        .ctrl_ce           (ctrl_ce),
        .pipe_sample_valid (pipe_sample_valid),

        .pipe_acc_x        (pipe_acc_x),
        .pipe_acc_y        (pipe_acc_y),
        .pipe_acc_z        (pipe_acc_z),

        .wake_mag_sq       (wake_mag_sq),       // Bus chia sẻ mag_sq sang stat_feature_extractor
        .wake_irq_raw      (wake_irq_raw),      // Tín hiệu vượt ngưỡng thô
        .wake_up_irq       (wake_up_irq)        // Đường ngắt Zero-Latency đi thẳng ra chân MCU GPIO
    );

    // =========================================================================
    // 3. KHỞI TẠO SUB-MODULE: cordic_attitude_estimator (ƯỚC LƯỢNG GÓC CORDIC)
    // =========================================================================
    cordic_attitude_estimator u_cordic_estimator (
        .clk               (clk),
        .rst_n             (rst_n),
        .ctrl_ce           (ctrl_ce),
        .pipe_sample_valid (pipe_sample_valid),
        .pipe_window_last  (pipe_window_last),

        .pipe_acc_x        (pipe_acc_x),
        .pipe_acc_y        (pipe_acc_y),
        .pipe_acc_z        (pipe_acc_z),

        .cordic_roll_out   (cordic_roll_out),   // Xuất Roll sang controller
        .cordic_pitch_out  (cordic_pitch_out),  // Xuất Pitch sang controller
        .cordic_done       (cordic_done)        // Báo hoàn tất sau 32 chu kỳ
    );

    // =========================================================================
    // 4. KHỞI TẠO SUB-MODULE: stat_feature_extractor (TRÍCH XUẤT ĐẶC TRƯNG THỐNG KÊ)
    // =========================================================================
    stat_feature_extractor u_stat_extractor (
        .clk               (clk),
        .rst_n             (rst_n),
        .ctrl_ce           (ctrl_ce),
        .pipe_sample_valid (pipe_sample_valid),
        .pipe_window_last  (pipe_window_last),  // Nhận cờ mẫu cuối cửa sổ từ controller

        .wake_mag_sq       (wake_mag_sq),       // Tái sử dụng tổng bình phương từ threshold detector

        .stat_rms_out      (stat_rms_out),      // Xuất RMS^2 sang controller
        .stat_var_out      (stat_var_out),      // Xuất Variance sang controller
        .stat_done         (stat_done)          // Báo hoàn tất sau 4 chu kỳ
    );

endmodule
