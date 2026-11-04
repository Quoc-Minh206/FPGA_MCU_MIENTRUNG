module fir_fsm (
    input  wire                 clk_50mhz,  // Xung nhịp tốc độ cao từ PLLVR
    input  wire                 rst_n,      // Reset hệ thống (Active Low)
    input  wire                 data_valid, // Cờ từ I2C Master báo có dữ liệu mới từ MPU6050
    
    // Dữ liệu thô từ cảm biến
    input  wire signed [15:0]   raw_ax,
    
    // Giao tiếp với IP Core DSP MULTADDALU
    output reg                  dsp_ce,
    output reg                  dsp_reset,
    output wire signed [15:0]   dsp_a,       // Đổi thành wire gán tổ hợp để chống lệch pha
    input  wire signed [53:0]   dsp_result,  // Nhận kết quả 54-bit từ MULTADDALU
    
    // Giao tiếp với ROM hệ số và điều khiển DSP
    output reg          [3:0]   tap_index,   // Quét từ 0 đến 14
    
    // Ngõ ra dữ liệu đã lọc sạch rung cơ học
    output reg  signed [15:0]   filtered_ax,
    output reg                  filter_done
);

    // Mảng thanh ghi trượt lưu 15 mẫu (Cửa sổ trượt 15-tap)
    reg signed [15:0] shift_ax [0:14];
    
    // Định nghĩa 4 trạng thái FSM
    localparam IDLE  = 2'd0;
    localparam SHIFT = 2'd1;
    localparam CALC  = 2'd2;
    localparam LATCH = 2'd3;
    
    reg [1:0] state;
    integer i;

    // Gán tổ hợp: Dữ liệu mẫu thay đổi đồng thời với tap_index và coef_in của ROM
    assign dsp_a = shift_ax[tap_index];

    always @(posedge clk_50mhz or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            tap_index   <= 4'd0;
            dsp_ce      <= 1'b0;
            dsp_reset   <= 1'b1;
            filter_done <= 1'b0;
            filtered_ax <= 16'sd0;
            for (i=0; i<15; i=i+1) shift_ax[i] <= 16'sd0;
        end else begin
            case (state)
                IDLE: begin
                    filter_done <= 1'b0;
                    dsp_ce      <= 1'b0;
                    if (data_valid) begin
                        state <= SHIFT; // Bắt đầu khi I2C có mẫu mới
                    end
                end

                SHIFT: begin
                    // Dịch chuyển mẫu cũ
                    for (i=14; i>0; i=i-1) shift_ax[i] <= shift_ax[i-1];
                    // Nạp mẫu mới vào đầu mảng
                    shift_ax[0] <= raw_ax;
                    
                    tap_index   <= 4'd0; // Bắt đầu từ tap 0
                    dsp_reset   <= 1'b0; // Giải phóng reset DSP
                    dsp_ce      <= 1'b1; // Bật DSP chạy
                    state       <= CALC;
                end

                CALC: begin
                    // Cạnh clock này chốt kết quả của tap_index hiện tại vào DSP
                    if (tap_index == 4'd14) begin
                        dsp_ce <= 1'b0;  // Đã nạp xong tap cuối cùng, tắt clock enable
                        state  <= LATCH; // Chuyển sang chốt dữ liệu
                    end else begin
                        tap_index <= tap_index + 1'b1;
                    end
                end

                LATCH: begin
                    // Định dạng Q1.15: Cắt 16 bit từ bit 30 xuống bit 15
                    filtered_ax <= dsp_result[30:15];
                    filter_done <= 1'b1; // Bật cờ báo hoàn thành lọc
                    state       <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule