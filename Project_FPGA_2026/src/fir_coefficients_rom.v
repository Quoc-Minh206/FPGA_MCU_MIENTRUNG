module fir_coefficients_rom (
    input  wire [3:0]        tap_index, // Chỉ số từ 0 đến 14 do FSM cấp
    output reg signed [15:0] coef_out   // Hệ số xuất ra cho MULTADDALU
);

    // 15 hệ số FIR Low-pass (Fs = 100Hz, Fc = 5Hz) đã lượng tử hóa 16-bit
    //Phải điều chỉnh lại khi lắp vào xe 
    localparam signed [15:0] COEF_0  = 16'sd144;
    localparam signed [15:0] COEF_1  = 16'sd310;
    localparam signed [15:0] COEF_2  = 16'sd789;
    localparam signed [15:0] COEF_3  = 16'sd1620;
    localparam signed [15:0] COEF_4  = 16'sd2698;
    localparam signed [15:0] COEF_5  = 16'sd3784;
    localparam signed [15:0] COEF_6  = 16'sd4593;
    localparam signed [15:0] COEF_7  = 16'sd4892; // Đỉnh trung tâm
    localparam signed [15:0] COEF_8  = 16'sd4593;
    localparam signed [15:0] COEF_9  = 16'sd3784;
    localparam signed [15:0] COEF_10 = 16'sd2698;
    localparam signed [15:0] COEF_11 = 16'sd1620;
    localparam signed [15:0] COEF_12 = 16'sd789;
    localparam signed [15:0] COEF_13 = 16'sd310;
    localparam signed [15:0] COEF_14 = 16'sd144;

    // Khối logic tổ hợp: Thay đổi output ngay lập tức khi tap_index thay đổi
    always @(*) begin
        case (tap_index)
            4'd0:  coef_out = COEF_0;
            4'd1:  coef_out = COEF_1;
            4'd2:  coef_out = COEF_2;
            4'd3:  coef_out = COEF_3;
            4'd4:  coef_out = COEF_4;
            4'd5:  coef_out = COEF_5;
            4'd6:  coef_out = COEF_6;
            4'd7:  coef_out = COEF_7;
            4'd8:  coef_out = COEF_8;
            4'd9:  coef_out = COEF_9;
            4'd10: coef_out = COEF_10;
            4'd11: coef_out = COEF_11;
            4'd12: coef_out = COEF_12;
            4'd13: coef_out = COEF_13;
            4'd14: coef_out = COEF_14;
            default: coef_out = 16'sd0; // Bắt buộc có default để tránh sinh ra chốt (latch)
        endcase
    end

endmodule