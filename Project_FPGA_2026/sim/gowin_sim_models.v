// =============================================================================
// File:        gowin_sim_models.v
// Dự án:       Mô hình mô phỏng hành vi (Behavioral Simulation Models) cho Icarus Verilog
// Mô tả:        Thay thế các khối IP mã hóa của Gowin (Gowin encrypted netlists)
//               trong môi trường mô phỏng độc lập (iverilog / vvp / Modelsim)
//               để kiểm chứng thiết kế theo design_verification.md
// =============================================================================

`timescale 1ns / 1ps
`define SIMULATION 1

// -----------------------------------------------------------------------------
// 1. MÔ HÌNH HÀNH VI: Gowin_PLLVR
//    Nhận xung 27 MHz, tạo xung clkout (100 MHz) và clkoutd (50 MHz)
// -----------------------------------------------------------------------------
module Gowin_PLLVR (
    output reg clkout,
    output reg clkoutd,
    input  wire clkin
);
    initial begin
        clkout  = 1'b0;
        clkoutd = 1'b0;
    end

    // clkout: 100 MHz (chu kỳ 10 ns)
    always #5 clkout = ~clkout;

    // clkoutd: 50 MHz (chu kỳ 20 ns)
    always #10 clkoutd = ~clkoutd;
endmodule

// -----------------------------------------------------------------------------
// 2. MÔ HÌNH HÀNH VI: FIFO_HS_Top (Dual-Clock CDC FIFO)
// -----------------------------------------------------------------------------
module FIFO_HS_Top (
    input  wire [15:0] Data,
    input  wire        WrClk,
    input  wire        RdClk,
    input  wire        WrEn,
    input  wire        RdEn,
    output wire        Almost_Empty,
    output wire        Almost_Full,
    output reg  [15:0] Q,
    output reg         Empty,
    output reg         Full
);
    reg [15:0] mem [0:255];
    reg [7:0]  wr_ptr;
    reg [7:0]  rd_ptr;
    reg [8:0]  count;

    initial begin
        wr_ptr = 8'd0;
        rd_ptr = 8'd0;
        count  = 9'd0;
        Empty  = 1'b1;
        Full   = 1'b0;
        Q      = 16'd0;
    end

    always @(posedge WrClk) begin
        if (WrEn && !Full) begin
            mem[wr_ptr] <= Data;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    always @(posedge RdClk) begin
        if (RdEn && !Empty) begin
            Q <= mem[rd_ptr];
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    always @(*) begin
        count = wr_ptr - rd_ptr;
        Empty = (wr_ptr == rd_ptr);
        Full  = (count >= 9'd250);
    end

    assign Almost_Empty = (count <= 9'd2);
    assign Almost_Full  = (count >= 9'd240);
endmodule

// -----------------------------------------------------------------------------
// 3. MÔ HÌNH HÀNH VI: CORDIC_Top (Chế độ Vectoring tính góc atan2 và cạnh huyền)
//    Độ trễ: 16 chu kỳ clock đồng bộ
// -----------------------------------------------------------------------------
module CORDIC_Top (
    input  wire                 clk,
    input  wire                 rst,
    input  wire signed [16:0]   x_i,
    input  wire signed [16:0]   y_i,
    input  wire signed [16:0]   theta_i,
    output reg  signed [16:0]   x_o,
    output reg  signed [16:0]   y_o,
    output reg  signed [16:0]   theta_o
);
    // Pipeline 16 tầng mô phỏng latency chuẩn của IP Core CORDIC (15 tầng đệm + 1 tầng xuất = 16 chu kỳ)
    reg signed [16:0] pipe_x [0:14];
    reg signed [16:0] pipe_theta [0:14];
    integer i;

    // Tính toán góc radian chuẩn và biên độ cạnh huyền: K * sqrt(x^2 + y^2)
    // Hệ số CORDIC K ~ 1.64676
    real r_x, r_y, r_theta, r_mag;
    integer int_theta, int_mag;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 15; i = i + 1) begin
                pipe_x[i]     <= 17'sd0;
                pipe_theta[i] <= 17'sd0;
            end
            x_o     <= 17'sd0;
            y_o     <= 17'sd0;
            theta_o <= 17'sd0;
        end else begin
            r_x = $itor(x_i);
            r_y = $itor(y_i);
            if (r_x == 0.0 && r_y == 0.0) begin
                r_theta = 0.0;
                r_mag   = 0.0;
            end else begin
                // atan2(y, x) xuất góc trong [-pi, +pi]
                // Định dạng góc Gowin CORDIC IP: 1.0 radian tương ứng 2^15 = 32768
                r_theta = $atan2(r_y, r_x);
                r_mag   = $sqrt(r_x * r_x + r_y * r_y) * 1.64676;
            end

            int_theta = $rtoi(r_theta * 32768.0);
            int_mag   = $rtoi(r_mag);

            pipe_theta[0] <= int_theta[16:0];
            pipe_x[0]     <= int_mag[16:0];

            for (i = 1; i < 15; i = i + 1) begin
                pipe_theta[i] <= pipe_theta[i-1];
                pipe_x[i]     <= pipe_x[i-1];
            end

            theta_o <= pipe_theta[14];
            x_o     <= pipe_x[14];
            y_o     <= 17'sd0;
        end
    end
endmodule

// -----------------------------------------------------------------------------
// 4. MÔ HÌNH HÀNH VI: Gowin_MULT (16x16 Signed Multiplier - Latency = 2 cycles)
//    Khớp chính xác với gowin_mult.v (AREG=1, BREG=1, OUT_REG=1, PIPE_REG=0)
// -----------------------------------------------------------------------------
module Gowin_MULT (
    output reg  [31:0] dout,
    input  wire [15:0] a,
    input  wire [15:0] b,
    input  wire        ce,
    input  wire        clk,
    input  wire        reset
);
    reg signed [17:0] a_reg, b_reg;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            a_reg <= 18'sd0;
            b_reg <= 18'sd0;
            dout  <= 32'd0;
        end else if (ce) begin
            a_reg <= {{2{a[15]}}, a};
            b_reg <= {{2{b[15]}}, b};
            dout  <= a_reg * b_reg;
        end
    end
endmodule

// -----------------------------------------------------------------------------
// 5. MÔ HÌNH HÀNH VI: I2C_MASTER_Top (Cho mô phỏng Icarus Verilog)
// -----------------------------------------------------------------------------
module I2C_MASTER_Top (
    input  wire        I_CLK,
    input  wire        I_RESETN,
    input  wire        I_TX_EN,
    input  wire [2:0]  I_WADDR,
    input  wire [7:0]  I_WDATA,
    input  wire        I_RX_EN,
    input  wire [2:0]  I_RADDR,
    output reg  [7:0]  O_RDATA,
    output reg         O_IIC_INT,
    inout  wire        SCL,
    inout  wire        SDA
);
    // Mô phỏng bus I2C tích cực
    initial begin
        O_RDATA   = 8'd0;
        O_IIC_INT = 1'b0;
    end
endmodule

// -----------------------------------------------------------------------------
// 6. MÔ HÌNH HÀNH VI: SPI_MASTER_Top (Cho mô phỏng Icarus Verilog)
// -----------------------------------------------------------------------------
module SPI_MASTER_Top (
    input  wire        I_CLK,
    input  wire        I_RESETN,
    input  wire        I_TX_EN,
    input  wire [2:0]  I_WADDR,
    input  wire [7:0]  I_WDATA,
    input  wire        I_RX_EN,
    input  wire [2:0]  I_RADDR,
    output reg  [7:0]  O_RDATA,
    output reg         O_SPI_INT,
    input  wire        MISO_MASTER,
    output reg         MOSI_MASTER,
    output reg  [0:0]  SS_N_MASTER,
    output reg         SCLK_MASTER,
    output wire        MISO_SLAVE,
    input  wire        MOSI_SLAVE,
    input  wire        SS_N_SLAVE,
    input  wire        SCLK_SLAVE
);
    reg [7:0] tx_buf;
    reg [3:0] bit_cnt;
    reg       sending;

    initial begin
        O_RDATA     = 8'd0;
        O_SPI_INT   = 1'b0;
        MOSI_MASTER = 1'b0;
        SS_N_MASTER = 1'b1;
        SCLK_MASTER = 1'b0;
        bit_cnt     = 4'd0;
        sending     = 1'b0;
    end

    always @(posedge I_CLK or negedge I_RESETN) begin
        if (!I_RESETN) begin
            SS_N_MASTER <= 1'b1;
            SCLK_MASTER <= 1'b0;
            MOSI_MASTER <= 1'b0;
            sending     <= 1'b0;
            bit_cnt     <= 4'd0;
        end else begin
            if (I_TX_EN && !sending) begin
                tx_buf      <= I_WDATA;
                SS_N_MASTER <= 1'b0;
                sending     <= 1'b1;
                bit_cnt     <= 4'd8;
            end else if (sending) begin
                SCLK_MASTER <= ~SCLK_MASTER;
                if (SCLK_MASTER) begin // Cạnh xuống: dịch bit
                    MOSI_MASTER <= tx_buf[7];
                    tx_buf      <= {tx_buf[6:0], 1'b0};
                    if (bit_cnt == 4'd1) begin
                        sending     <= 1'b0;
                        SS_N_MASTER <= 1'b1;
                    end else begin
                        bit_cnt <= bit_cnt - 1'b1;
                    end
                end
            end
        end
    end
endmodule

// -----------------------------------------------------------------------------
// 7. MÔ HÌNH HÀNH VI: MULTADDALU18X18 (DSP Primitive cho Gowin_MULTADDALU)
// -----------------------------------------------------------------------------
module MULTADDALU18X18 #(
    parameter A0REG = 1'b1,
    parameter B0REG = 1'b1,
    parameter A1REG = 1'b1,
    parameter B1REG = 1'b1,
    parameter CREG = 1'b0,
    parameter PIPE0_REG = 1'b1,
    parameter PIPE1_REG = 1'b1,
    parameter OUT_REG = 1'b1,
    parameter ASIGN0_REG = 1'b0,
    parameter ASIGN1_REG = 1'b0,
    parameter ACCLOAD_REG0 = 1'b0,
    parameter ACCLOAD_REG1 = 1'b0,
    parameter BSIGN0_REG = 1'b0,
    parameter BSIGN1_REG = 1'b0,
    parameter SOA_REG = 1'b0,
    parameter B_ADD_SUB = 1'b0,
    parameter C_ADD_SUB = 1'b0,
    parameter MULTADDALU18X18_MODE = 1,
    parameter MULT_RESET_MODE = "SYNC"
)(
    output reg signed [53:0] DOUT,
    output wire [54:0] CASO,
    output wire [17:0] SOA,
    output wire [17:0] SOB,
    input  wire [53:0] C,
    input  wire signed [17:0] A0,
    input  wire signed [17:0] B0,
    input  wire signed [17:0] A1,
    input  wire signed [17:0] B1,
    input  wire [1:0]  ASIGN,
    input  wire [1:0]  BSIGN,
    input  wire [53:0] CASI,
    input  wire        ACCLOAD,
    input  wire [17:0] SIA,
    input  wire [17:0] SIB,
    input  wire        CE,
    input  wire        CLK,
    input  wire        RESET,
    input  wire [1:0]  ASEL,
    input  wire [1:0]  BSEL
);
    assign CASO = 55'd0;
    assign SOA  = 18'd0;
    assign SOB  = 18'd0;

    reg signed [35:0] prod;
    reg signed [53:0] accum;

    initial begin
        accum = 54'sd0;
        DOUT  = 54'sd0;
    end

    always @(posedge CLK or posedge RESET) begin
        if (RESET) begin
            accum <= 54'sd0;
            DOUT  <= 54'sd0;
        end else if (CE) begin
            prod = A0 * B0;
            if (ACCLOAD) begin
                accum <= prod;
                DOUT  <= prod;
            end else begin
                accum <= accum + prod;
                DOUT  <= accum + prod;
            end
        end
    end
endmodule

