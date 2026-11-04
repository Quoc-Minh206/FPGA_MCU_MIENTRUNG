// =============================================================================
// Tên Module:   accel_threshold_detector
// Dự án:        Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Phân hệ:      FPGA Math Core & Feature Extraction Engine
// Nền tảng:     Gowin LittleBee GW1NSR-LV4CQN48PC6/I5 (OneKiwi 4K)
// Kiến trúc:    SỬ DỤNG GOWIN HARD IP CORE (Gowin_MULT - MULT18X18 DSP Blocks)
// Mô tả:        Khối phát hiện ngưỡng chấn động va chạm thời gian thực (Zero-Latency),
//               sử dụng 3 khối Gowin_MULT IP Core để tính Ax^2, Ay^2, Az^2,
//               bộ cộng mở rộng 34-bit, bộ đếm lọc nhiễu nhọn (Persistency Filter)
//               và bus chia sẻ mag_sq cho khối thống kê.
// =============================================================================

`timescale 1ns / 1ps

module accel_threshold_detector #(
    // -------------------------------------------------------------------------
    // CÁC THAM SỐ CẤU HÌNH (HARDWARE PARAMETERS)
    // -------------------------------------------------------------------------
    // Ngưỡng năng lượng gia tốc bình phương: 2.5g = 10240 LSB (Q4.12)
    // (10240)^2 = 104,857,600 LSB^2 = 34'h0_0640_0000.
    parameter [33:0] CONF_ACC_THRESHOLD_SQ = 34'h0_0640_0000,

    // Số mẫu liên tiếp vượt ngưỡng để khẳng định va chạm (chống nhiễu nhọn - Spike Noise)
    parameter [2:0]  CONF_WAKEUP_PERSIST   = 3'd3
)(
    // -------------------------------------------------------------------------
    // CỔNG GIAO TIẾP HỆ THỐNG & ĐIỀU KHIỂN (SYSTEM & CONTROL PORTS)
    // -------------------------------------------------------------------------
    input  wire        clk,               // Xung nhịp hệ thống 50 MHz từ PLLVR
    input  wire        rst_n,             // Reset toàn hệ thống, tích cực mức thấp (Active-Low)
    input  wire        ctrl_ce,           // Tín hiệu Clock-Enable từ math_pipeline_ctrl
    input  wire        pipe_sample_valid, // Xung 1 chu kỳ clock báo mẫu gia tốc mới hợp lệ

    // -------------------------------------------------------------------------
    // DỮ LIỆU GIA TỐC ĐẦU VÀO TỪ math_pipeline_ctrl (INPUT ACCEL DATA)
    // -------------------------------------------------------------------------
    input  wire signed [15:0] pipe_acc_x, // Gia tốc trục X sau lọc FIR (Signed Q4.12)
    input  wire signed [15:0] pipe_acc_y, // Gia tốc trục Y sau lọc FIR (Signed Q4.12)
    input  wire signed [15:0] pipe_acc_z, // Gia tốc trục Z sau lọc FIR (Signed Q4.12)

    // -------------------------------------------------------------------------
    // TÍN HIỆU ĐẦU RA (OUTPUT SIGNALS)
    // -------------------------------------------------------------------------
    output wire [33:0] wake_mag_sq,       // Tổng bình phương Ax^2 + Ay^2 + Az^2 (Unsigned Q8.24)
    output wire        wake_irq_raw,      // Cờ vượt ngưỡng thô tức thời
    output reg         wake_up_irq        // Xung ngắt phần cứng chính thức gửi tới MCU GPIO (Sonix SN32F407)
);

    // Reset tích cực cao cấp cho Gowin Hard IP Core
    wire ip_rst = ~rst_n;

    // Đường bus ngõ ra 32-bit từ 3 khối Gowin_MULT IP Core
    wire [31:0] ax_sq_32;
    wire [31:0] ay_sq_32;
    wire [31:0] az_sq_32;

    // =========================================================================
    // 1. KHỞI TẠO 3 IP CORE: Gowin_MULT (TÍNH Ax^2, Ay^2, Az^2)
    // =========================================================================
    // Mỗi Gowin_MULT có trễ 2 chu kỳ clock (AREG=1, OUT_REG=1)
    Gowin_MULT u_mult_ax (
        .dout  (ax_sq_32),
        .a     (pipe_acc_x),
        .b     (pipe_acc_x),
        .ce    (ctrl_ce),
        .clk   (clk),
        .reset (ip_rst)
    );

    Gowin_MULT u_mult_ay (
        .dout  (ay_sq_32),
        .a     (pipe_acc_y),
        .b     (pipe_acc_y),
        .ce    (ctrl_ce),
        .clk   (clk),
        .reset (ip_rst)
    );

    Gowin_MULT u_mult_az (
        .dout  (az_sq_32),
        .a     (pipe_acc_z),
        .b     (pipe_acc_z),
        .ce    (ctrl_ce),
        .clk   (clk),
        .reset (ip_rst)
    );

    // =========================================================================
    // 2. GIÓNG HÀNG ĐỘ TRỄ VALID (2-STAGE DELAY LINE CHO GOWIN_MULT)
    // =========================================================================
    reg [1:0] mult_valid_pipe;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mult_valid_pipe <= 2'b00;
        end else if (ctrl_ce) begin
            mult_valid_pipe <= {mult_valid_pipe[0], pipe_sample_valid};
        end
    end

    wire mult_done = mult_valid_pipe[1]; // Dữ liệu bình phương hợp lệ sau 2 chu kỳ

    // =========================================================================
    // 3. STAGE CỘNG TỔNG 3 TRỤC VÀ SO SÁNH NGƯỠNG (ACCUMULATE & COMPARE)
    // =========================================================================
    reg [33:0] mag_sq_r;
    reg        thresh_hit_r;
    reg        valid_sum_r;

    // shortcut: single 34-bit adder shared between accumulator and threshold comparator
    wire [33:0] mag_sq_sum = {2'd0, ax_sq_32} + {2'd0, ay_sq_32} + {2'd0, az_sq_32};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_sq_r     <= 34'd0;
            thresh_hit_r <= 1'b0;
            valid_sum_r  <= 1'b0;
        end else if (ctrl_ce) begin
            valid_sum_r <= mult_done;
            if (mult_done) begin
                mag_sq_r     <= mag_sq_sum;
                thresh_hit_r <= (mag_sq_sum >= CONF_ACC_THRESHOLD_SQ);
            end
        end
    end

    assign wake_mag_sq  = mag_sq_r;
    assign wake_irq_raw = thresh_hit_r;

    // =========================================================================
    // 4. BỘ ĐẾM LỌC NHIỄU NHỌN (PERSISTENCY FILTER)
    // =========================================================================
    reg [2:0] persist_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            persist_cnt <= 3'd0;
            wake_up_irq <= 1'b0;
        end else if (ctrl_ce) begin
            if (valid_sum_r) begin
                if (thresh_hit_r) begin
                    if (persist_cnt < CONF_WAKEUP_PERSIST) begin
                        persist_cnt <= persist_cnt + 3'd1;
                    end
                    // Đạt đủ số mẫu liên tiếp vượt ngưỡng thì kích hoạt ngắt
                    if (persist_cnt >= (CONF_WAKEUP_PERSIST - 3'd1)) begin
                        wake_up_irq <= 1'b1;
                    end
                end else begin
                    persist_cnt <= 3'd0;
                    wake_up_irq <= 1'b0;
                end
            end
        end
    end

endmodule
