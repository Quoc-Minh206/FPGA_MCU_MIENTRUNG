// =============================================================================
// Tên Module:   cordic_attitude_estimator
// Dự án:        Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Phân hệ:      FPGA Math Core & Feature Extraction Engine
// Nền tảng:     Gowin LittleBee GW1NSR-LV4CQN48PC6/I5 (OneKiwi 4K)
// Kiến trúc:    SỬ DỤNG GOWIN CORDIC IP CORE (CORDIC_Top - Vectoring Mode)
// Mô tả:        Khối ước lượng góc nghiêng Roll và Pitch sử dụng 2 lõi Gowin CORDIC
//               IP Core nối tiếp (Cascaded CORDIC Pipeline 32 chu kỳ).
//               CORDIC 1 tính Roll = atan2(Ay, Az) và biên độ cạnh huyền Hypot_yz.
//               CORDIC 2 tính Pitch = atan2(-Ax, Hypot_yz).
//               Tích hợp mạch đổi chuẩn Radian sang Độ Signed Q8.8.
// =============================================================================

`timescale 1ns / 1ps

module cordic_attitude_estimator (
    // -------------------------------------------------------------------------
    // CỔNG GIAO TIẾP HỆ THỐNG & ĐIỀU KHIỂN (SYSTEM & CONTROL PORTS)
    // -------------------------------------------------------------------------
    input  wire               clk,               // Xung nhịp hệ thống 50 MHz
    input  wire               rst_n,             // Reset tích cực mức thấp (Active-Low)
    input  wire               ctrl_ce,           // Tín hiệu Clock-Enable cho phép pipeline chạy
    input  wire               pipe_sample_valid, // Xung 1 chu kỳ clock báo mẫu IMU mới bắt đầu nạp vào
    input  wire               pipe_window_last,  // Cờ báo mẫu cuối cùng của cửa sổ N = 32 (mẫu số 31)

    // -------------------------------------------------------------------------
    // DỮ LIỆU GIA TỐC ĐẦU VÀO TỪ math_pipeline_ctrl (INPUT ACCEL DATA)
    // -------------------------------------------------------------------------
    input  wire signed [15:0] pipe_acc_x,        // Gia tốc trục X (Signed Q4.12)
    input  wire signed [15:0] pipe_acc_y,        // Gia tốc trục Y (Signed Q4.12)
    input  wire signed [15:0] pipe_acc_z,        // Gia tốc trục Z (Signed Q4.12)

    // -------------------------------------------------------------------------
    // DỮ LIỆU GÓC ĐẦU RA (OUTPUT ATTITUDE ANGLES)
    // -------------------------------------------------------------------------
    output reg  signed [15:0] cordic_roll_out,   // Góc nghiêng Roll (Signed Q8.8, dải [-180.00, +180.00] độ)
    output reg  signed [15:0] cordic_pitch_out,  // Góc dốc Pitch (Signed Q8.8, dải [-90.00, +90.00] độ)
    output reg                cordic_done        // Xung 1 chu kỳ clock báo cả 2 góc đã tính xong
);

    wire ip_rst = ~rst_n;

    // =========================================================================
    // 1. TIỀN XỬ LÝ GÓC PHẦN TƯ CHO CORDIC 1 (QUADRANT PRE-CONDITIONING)
    // =========================================================================
    // CORDIC vectoring yêu cầu X >= 0 để đảm bảo hội tụ góc.
    // Nếu Az < 0: Ta lật dấu X = -Az, Y = -Ay, và ghi nhớ cờ az_neg để bù lại 180 độ.
    // Mở rộng 16-bit sang 17-bit: {data[15], data} để giữ nguyên biên độ cạnh huyền đồng bộ với CORDIC 2
    wire signed [16:0] az_ext = {pipe_acc_z[15], pipe_acc_z};
    wire signed [16:0] ay_ext = {pipe_acc_y[15], pipe_acc_y};

    wire az_is_neg = (pipe_acc_z < 16'sd0);
    wire ay_is_neg = (pipe_acc_y < 16'sd0);

    wire signed [16:0] c1_x_in = az_is_neg ? -az_ext : az_ext;
    wire signed [16:0] c1_y_in = az_is_neg ? -ay_ext : ay_ext;

    // =========================================================================
    // 2. KHỞI TẠO IP CORE: CORDIC 1 (TÍNH ROLL & BIÊN ĐỘ CẠNH HUYỀN HYPOT_YZ)
    // =========================================================================
    wire signed [16:0] c1_x_out;      // Cạnh huyền K * sqrt(Ay^2 + Az^2)
    wire signed [16:0] c1_y_out;      // Dư số Y (~ 0)
    wire signed [16:0] c1_theta_out;  // Góc pha Radian trong [-pi/2, +pi/2]

    CORDIC_Top u_cordic1_roll (
        .clk      (clk),
        .rst      (ip_rst),
        .x_i      (c1_x_in),
        .y_i      (c1_y_in),
        .theta_i  (17'sd0),
        .x_o      (c1_x_out),
        .y_o      (c1_y_out),
        .theta_o  (c1_theta_out)
    );

    // =========================================================================
    // 3. MẢNG THANH GHI TRỄ 16 CHU KỲ (ĐỒNG BỘ THEO LATENCY CỦA CORDIC 1)
    // =========================================================================
    reg signed [15:0] ax_delay_pipe  [0:15];
    reg               az_neg_pipe    [0:15];
    reg               ay_neg_pipe    [0:15];
    reg        [15:0] c1_valid_pipe;
    reg        [15:0] c1_last_pipe;
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 16; i = i + 1) begin
                ax_delay_pipe[i] <= 16'sd0;
                az_neg_pipe[i]   <= 1'b0;
                ay_neg_pipe[i]   <= 1'b0;
            end
            c1_valid_pipe <= 16'd0;
            c1_last_pipe  <= 16'd0;
        end else if (ctrl_ce) begin
            // Shift pipeline cho Ax và tín hiệu điều khiển
            ax_delay_pipe[0] <= pipe_acc_x;
            az_neg_pipe[0]   <= az_is_neg;
            ay_neg_pipe[0]   <= ay_is_neg;
            c1_valid_pipe[0] <= pipe_sample_valid;
            c1_last_pipe[0]  <= pipe_window_last;

            for (i = 1; i < 16; i = i + 1) begin
                ax_delay_pipe[i] <= ax_delay_pipe[i - 1];
                az_neg_pipe[i]   <= az_neg_pipe[i - 1];
                ay_neg_pipe[i]   <= ay_neg_pipe[i - 1];
                c1_valid_pipe[i] <= c1_valid_pipe[i - 1];
                c1_last_pipe[i]  <= c1_last_pipe[i - 1];
            end
        end
    end

    // =========================================================================
    // 4. CHUYỂN ĐỔI GÓC CORDIC 1 SANG ĐỘ Q8.8 VÀ HIỆU CHỈNH 4 GÓC PHẦN TƯ
    // =========================================================================
    // Hệ số đổi Radian (1.0 rad = 32768) sang Độ Q8.8 (1 độ = 256):
    // 180 * 256 / pi = 14667.6 -> 14667
    wire signed [31:0] c1_deg_mult = c1_theta_out * 16'sd14667;
    wire signed [15:0] c1_deg_base = c1_deg_mult[30:15]; // Dùng cho Roll ở chu kỳ 16

    // Hiệu chỉnh góc 180 độ: 180 * 256 = 46080 (16'sd46080)
    reg signed [15:0] c1_roll_deg_final;

    always @(*) begin
        if (!az_neg_pipe[15]) begin
            c1_roll_deg_final = c1_deg_base;
        end else begin
            if (!ay_neg_pipe[15]) begin
                c1_roll_deg_final = 16'sd46080 - c1_deg_base;  // Góc phần tư thứ II
            end else begin
                c1_roll_deg_final = -16'sd46080 - c1_deg_base; // Góc phần tư thứ III
            end
        end
    end

    // =========================================================================
    // 5. CHUẨN BỊ ĐẦU VÀO CHO CORDIC 2: PITCH = atan2(-Ax * K, Hypot_yz)
    // =========================================================================
    // Hypot_yz từ CORDIC 1 đã có sẵn hệ số K ~ 1.64676.
    // Ta nhân -Ax với K (1.64676 * 16384 = 26980) để đồng bộ tỉ lệ trước khi vào CORDIC 2.
    wire signed [31:0] ax_scaled_32 = (-ax_delay_pipe[15]) * 16'sd26980;
    wire signed [16:0] c2_y_in      = ax_scaled_32[30:14]; // 17-bit có dấu
    wire signed [16:0] c2_x_in      = c1_x_out;            // Cạnh huyền (luôn >= 0)

    // =========================================================================
    // 6. KHỞI TẠO IP CORE: CORDIC 2 (TÍNH PITCH)
    // =========================================================================
    wire signed [16:0] c2_x_out;
    wire signed [16:0] c2_y_out;
    wire signed [16:0] c2_theta_out; // Góc Pitch radian trong [-pi/2, +pi/2]

    CORDIC_Top u_cordic2_pitch (
        .clk      (clk),
        .rst      (ip_rst),
        .x_i      (c2_x_in),
        .y_i      (c2_y_in),
        .theta_i  (17'sd0),
        .x_o      (c2_x_out),
        .y_o      (c2_y_out),
        .theta_o  (c2_theta_out)
    );

    // =========================================================================
    // 7. THANH GHI TRỄ ROLL CHỜ PITCH (16 CHU KỲ TRONG KHI CORDIC 2 TÍNH)
    // =========================================================================
    reg signed [15:0] roll_delay_pipe [0:15];
    reg        [15:0] c2_valid_pipe;
    reg        [15:0] c2_last_pipe;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 16; i = i + 1) begin
                roll_delay_pipe[i] <= 16'sd0;
            end
            c2_valid_pipe <= 16'd0;
            c2_last_pipe  <= 16'd0;
        end else if (ctrl_ce) begin
            roll_delay_pipe[0] <= c1_roll_deg_final;
            c2_valid_pipe[0]   <= c1_valid_pipe[15];
            c2_last_pipe[0]    <= c1_last_pipe[15];

            for (i = 1; i < 16; i = i + 1) begin
                roll_delay_pipe[i] <= roll_delay_pipe[i - 1];
                c2_valid_pipe[i]   <= c2_valid_pipe[i - 1];
                c2_last_pipe[i]    <= c2_last_pipe[i - 1];
            end
        end
    end

    // Đổi Pitch từ Radian sang Độ Signed Q8.8
    wire signed [31:0] c2_deg_mult = c2_theta_out * 16'sd14667;
    wire signed [15:0] c2_pitch_deg_final = c2_deg_mult[30:15];

    // =========================================================================
    // 8. XUẤT ĐỒNG BỘ 2 GÓC ROLL & PITCH Ở CHU KỲ 32
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cordic_roll_out  <= 16'sd0;
            cordic_pitch_out <= 16'sd0;
            cordic_done      <= 1'b0;
        end else if (ctrl_ce) begin
            // Chỉ phát xung hoàn tất khi mẫu cuối cùng của cửa sổ (mẫu 31) hoàn tất CORDIC
            cordic_done <= c2_last_pipe[15];
            if (c2_valid_pipe[15]) begin
                cordic_roll_out  <= roll_delay_pipe[15];
                cordic_pitch_out <= c2_pitch_deg_final;
            end
        end
    end

endmodule
