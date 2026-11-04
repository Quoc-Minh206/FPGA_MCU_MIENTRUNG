// =============================================================================
// Tên Module:   stat_feature_extractor
// Dự án:        Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Phân hệ:      FPGA Math Core & Feature Extraction Engine
// Nền tảng:     Gowin LittleBee GW1NSR-LV4CQN48PC6/I5 (OneKiwi 4K)
// Kiến trúc:    SỬ DỤNG GOWIN HARD IP CORE (Gowin_MULT - MULT18X18 DSP Blocks)
// Mô tả:        Khối trích xuất vector 2 đặc trưng thống kê trên cửa sổ N = 32 mẫu:
//               1. Mean Square (RMS^2): Năng lượng hiệu dụng trung bình E[X^2].
//               2. Variance: Phương sai dao động xung quanh kỳ vọng trung bình.
//               Sử dụng 2 khối Gowin_MULT IP Core để tính bình phương mẫu xi^2
//               và bình phương kỳ vọng trung bình mu^2.
// =============================================================================

`timescale 1ns / 1ps

module stat_feature_extractor (
    // -------------------------------------------------------------------------
    // CỔNG GIAO TIẾP HỆ THỐNG & ĐIỀU KHIỂN (SYSTEM & CONTROL PORTS)
    // -------------------------------------------------------------------------
    input  wire        clk,               // Xung nhịp hệ thống 50 MHz
    input  wire        rst_n,             // Reset tích cực mức thấp (Active-Low)
    input  wire        ctrl_ce,           // Tín hiệu Clock-Enable cho phép tính toán
    input  wire        pipe_sample_valid, // Xung 1 chu kỳ clock báo mẫu năng lượng mới hợp lệ
    input  wire        pipe_window_last,  // Xung 1 chu kỳ clock báo mẫu thứ 32 (mẫu cuối cửa sổ N=32)

    // -------------------------------------------------------------------------
    // DỮ LIỆU ĐẦU VÀO TỪ accel_threshold_detector (INPUT ENERGY DATA)
    // -------------------------------------------------------------------------
    input  wire [33:0] wake_mag_sq,       // Tổng bình phương Ax^2 + Ay^2 + Az^2 (34-bit, Q8.24)

    // -------------------------------------------------------------------------
    // DỮ LIỆU ĐẶC TRƯNG ĐẦU RA (OUTPUT STATISTICAL FEATURES)
    // -------------------------------------------------------------------------
    output reg  [15:0] stat_rms_out,      // Năng lượng hiệu dụng RMS^2 = E[X^2] (Unsigned Q8.8)
    output reg  [15:0] stat_var_out,      // Phương sai dao động Variance (Unsigned Q12.4)
    output reg         stat_done          // Xung 1 chu kỳ clock báo cả 2 đặc trưng đã hoàn tất
);

    wire ip_rst = ~rst_n;

    // Trích xuất 16-bit [31:16] tương ứng với định dạng Unsigned Q8.8 để tính toán
    wire [15:0] sample_x = wake_mag_sq[31:16];

    // =========================================================================
    // 1. KHỞI TẠO IP CORE: Gowin_MULT 1 (TÍNH BÌNH PHƯƠNG MẪU xi * xi)
    // =========================================================================
    wire [31:0] sample_sq_w;

    Gowin_MULT u_mult_sample (
        .dout  (sample_sq_w),
        .a     (sample_x),
        .b     (sample_x),
        .ce    (ctrl_ce),
        .clk   (clk),
        .reset (ip_rst)
    );

    // Đường trễ 3 chu kỳ đồng bộ với accel_threshold_detector (Gowin_MULT: 2 chu kỳ + adder/reg: 1 chu kỳ)
    reg [2:0] pre_valid_pipe;
    reg [2:0] pre_last_pipe;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pre_valid_pipe <= 3'b000;
            pre_last_pipe  <= 3'b000;
        end else if (ctrl_ce) begin
            pre_valid_pipe <= {pre_valid_pipe[1:0], pipe_sample_valid};
            pre_last_pipe  <= {pre_last_pipe[1:0], pipe_window_last};
        end
    end

    wire mag_valid = pre_valid_pipe[2];
    wire mag_last  = pre_last_pipe[2];

    // Đường trễ 2 chu kỳ để gióng hàng sample_x và cờ điều khiển với Gowin_MULT
    reg [15:0] sample_x_d1, sample_x_d2;
    reg        valid_d1, valid_d2;
    reg        last_d1, last_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_x_d1 <= 16'd0;
            sample_x_d2 <= 16'd0;
            valid_d1    <= 1'b0;
            valid_d2    <= 1'b0;
            last_d1     <= 1'b0;
            last_d2     <= 1'b0;
        end else if (ctrl_ce) begin
            sample_x_d1 <= sample_x;
            sample_x_d2 <= sample_x_d1;
            valid_d1    <= mag_valid;
            valid_d2    <= valid_d1;
            last_d1     <= mag_last;
            last_d2     <= last_d1;
        end
    end

    // =========================================================================
    // 2. TÍCH LŨY MẪU CỬA SỔ N = 32 (SAMPLE ACCUMULATION)
    // =========================================================================
    reg [20:0] sum_linear; // Tích lũy tuyến tính: Sum(xi)
    reg [37:0] sum_square; // Tích lũy bình phương: Sum(xi^2)
    wire [37:0] sum_sq_accum = sum_square + {6'd0, sample_sq_w};

    reg [15:0] mean_mu_r1;
    reg [31:0] ex2_mean_sq_r1;
    reg        post_step1_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_linear       <= 21'd0;
            sum_square       <= 38'd0;
            mean_mu_r1       <= 16'd0;
            ex2_mean_sq_r1   <= 32'd0;
            post_step1_valid <= 1'b0;
        end else if (ctrl_ce) begin
            post_step1_valid <= 1'b0;

            if (valid_d2) begin
                sum_linear <= sum_linear + {5'd0, sample_x_d2};
                sum_square <= sum_square + {6'd0, sample_sq_w};
            end

            // Khi mẫu cuối cùng của cửa sổ đến
            if (last_d2) begin
                // Chia 32 bằng phép dịch bit: sum >> 5
                mean_mu_r1       <= (sum_linear + {5'd0, sample_x_d2}) >> 5;
                ex2_mean_sq_r1   <= sum_sq_accum[36:5];
                post_step1_valid <= 1'b1;

                // Reset bộ tích lũy cho cửa sổ tiếp theo
                sum_linear       <= 21'd0;
                sum_square       <= 38'd0;
            end
        end
    end

    // =========================================================================
    // 3. KHỞI TẠO IP CORE: Gowin_MULT 2 (TÍNH BÌNH PHƯƠNG KỲ VỌNG mu * mu)
    // =========================================================================
    wire [31:0] mu_square_w;

    Gowin_MULT u_mult_mu (
        .dout  (mu_square_w),
        .a     (mean_mu_r1),
        .b     (mean_mu_r1),
        .ce    (ctrl_ce),
        .clk   (clk),
        .reset (ip_rst)
    );

    // Gióng hàng trễ 2 chu kỳ cho phép nhân mu * mu
    reg [31:0] ex2_latched_d1, ex2_latched_d2;
    reg        post_step2_valid, post_step3_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex2_latched_d1   <= 32'd0;
            ex2_latched_d2   <= 32'd0;
            post_step2_valid <= 1'b0;
            post_step3_valid <= 1'b0;
        end else if (ctrl_ce) begin
            ex2_latched_d1   <= ex2_mean_sq_r1;
            ex2_latched_d2   <= ex2_latched_d1;
            post_step2_valid <= post_step1_valid;
            post_step3_valid <= post_step2_valid;
        end
    end

    // =========================================================================
    // 4. PHÉP TRỪ TÍNH PHƯƠNG SAI & MẠCH BÃO HÒA CHỐNG ÂM
    // =========================================================================
    reg signed [32:0] var_diff_r;
    reg        [15:0] rms_latched_r;
    reg               post_step4_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            var_diff_r       <= 33'sd0;
            rms_latched_r    <= 16'd0;
            post_step4_valid <= 1'b0;
        end else if (ctrl_ce) begin
            post_step4_valid <= post_step3_valid;
            if (post_step3_valid) begin
                // Var = E[X^2] - mu^2
                var_diff_r    <= $signed({1'b0, ex2_latched_d2}) - $signed({1'b0, mu_square_w});
                rms_latched_r <= ex2_latched_d2[15:0]; // Chốt 16-bit Q8.8
            end
        end
    end

    // Giai đoạn xuất kết quả chính thức
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_rms_out <= 16'd0;
            stat_var_out <= 16'd0;
            stat_done    <= 1'b0;
        end else if (ctrl_ce) begin
            stat_done <= post_step4_valid;
            if (post_step4_valid) begin
                stat_rms_out <= rms_latched_r;

                // shortcut: if sign bit is 1 (negative) clamp to 0; if 0, var_diff[15:0] is naturally 0
                if (var_diff_r[32]) begin
                    stat_var_out <= 16'h0000;
                end else if (|var_diff_r[31:16]) begin
                    stat_var_out <= 16'hFFFF;
                end else begin
                    stat_var_out <= var_diff_r[15:0];
                end
            end
        end
    end

endmodule
