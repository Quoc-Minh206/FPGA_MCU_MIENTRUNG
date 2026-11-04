// =============================================================================
// File:        Clock_manager.v
// Dự án:       Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Mô tả:       Bộ tạo xung nhịp đồng bộ 50 MHz duy nhất từ thạch anh 27 MHz
//               sử dụng Hard IP Gowin_PLLVR. Loại bỏ hoàn toàn các miền xung thừa.
// =============================================================================

`timescale 1ns / 1ps

module Clock_manager (
    input  wire clk,        // Xung nhịp thạch anh 27 MHz onboard
    input  wire rst_n,      // Reset tích cực mức thấp
    output wire clk_50MHz   // Xung nhịp hệ thống toàn cục 50 MHz duy nhất
);

    wire clk_100mhz_internal;

    Gowin_PLLVR u_pllvr (
        .clkin   (clk),
        .clkout  (clk_100mhz_internal), // 100 MHz nội bộ của PLL
        .clkoutd (clk_50MHz)            // 50 MHz ra toàn bộ hệ thống FPGA
    );

endmodule
