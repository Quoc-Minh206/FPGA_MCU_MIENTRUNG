module fir_top_module (
    input  wire                 clk_50mhz,  // Từ IP Core PLLVR
    input  wire                 rst_n,
    input  wire                 data_valid,
    input  wire signed [15:0]   raw_ax,
    output wire signed [15:0]   filtered_ax,
    output wire                 filter_done
);

    // --- CÁC ĐƯỜNG DÂY LIÊN KẾT ---
    wire                 w_dsp_ce;
    wire                 w_dsp_reset;
    wire signed [15:0]   w_dsp_a;
    wire signed [15:0]   w_coef_in; 
    wire signed [53:0]   w_dsp_result; // Chuẩn 54-bit cho Accumulator của Gowin
    wire        [3:0]    w_tap_index;
    wire                 w_dsp_load;

    // accload = 1 khi ở tap 0 (nạp tích đầu tiên), = 0 từ tap 1..14 (cộng dồn)
    assign w_dsp_load = (w_tap_index == 4'd0);

    // --- MODULE 3: MÁY TRẠNG THÁI FSM ---
    fir_fsm fsm_inst (
        .clk_50mhz   (clk_50mhz),
        .rst_n       (rst_n),
        .data_valid  (data_valid),
        .raw_ax      (raw_ax),
        .dsp_ce      (w_dsp_ce),
        .dsp_reset   (w_dsp_reset),
        .dsp_a       (w_dsp_a),
        .dsp_result  (w_dsp_result), // Đưa 54-bit vào FSM để chốt bit [30:15]
        .tap_index   (w_tap_index),
        .filtered_ax (filtered_ax),
        .filter_done (filter_done)
    );

    // --- MODULE 2: ROM HỆ SỐ ---
    fir_coefficients_rom rom_inst (
        .tap_index   (w_tap_index),
        .coef_out    (w_coef_in)
    );

    // --- MODULE 1: IP CORE DSP MULTADDALU ---
    Gowin_MULTADDALU dsp_inst (
        .dout        (w_dsp_result),        // Ngõ ra 54-bit
        .caso        (),                    // Để trống vì không ghép tầng DSP
        .a0          (w_dsp_a),             // Mẫu dữ liệu thô (16-bit)
        .b0          (w_coef_in),           // Hệ số lọc FIR từ ROM (16-bit)
        .a1          (16'sd0),              // Khóa bằng 0
        .b1          (16'sd0),              // Khóa bằng 0
        .accload     (w_dsp_load),          // 1: Load tap 0 | 0: Cộng dồn tap 1..14
        .ce          (w_dsp_ce),            // Clock Enable từ FSM
        .clk         (clk_50mhz),
        .reset       (w_dsp_reset)          // Reset đồng bộ
    );

endmodule