// =============================================================================
// File:        uart_tx.v
// Dự án:       Kiểm thử cảm biến MPU6050 & FPGA qua giao tiếp UART
// Mô tả:        Bộ phát UART (Transmitter) chuẩn 8-N-1 (8 data bits, no parity, 1 stop bit)
//               Tần số xung nhịp: 50 MHz, Tốc độ Baud mặc định: 115200 bps
// =============================================================================

`timescale 1ns / 1ps

module uart_tx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx_pin,
    output wire       tx_busy
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE; // 50,000,000 / 115,200 = 434 chu kỳ

    localparam [2:0]
        ST_IDLE  = 3'd0,
        ST_START = 3'd1,
        ST_DATA  = 3'd2,
        ST_STOP  = 3'd3;

    reg [2:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  data_buf;

    assign tx_busy = (state != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            clk_cnt  <= 16'd0;
            bit_idx  <= 3'd0;
            tx_pin   <= 1'b1; // Đường truyền rảnh ở mức cao (High)
            data_buf <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx_pin  <= 1'b1;
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (tx_start) begin
                        data_buf <= tx_data;
                        state    <= ST_START;
                    end
                end

                ST_START: begin
                    tx_pin <= 1'b0; // Start bit mức 0
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end else begin
                        clk_cnt <= 16'd0;
                        state   <= ST_DATA;
                    end
                end

                ST_DATA: begin
                    tx_pin <= data_buf[bit_idx]; // Gửi lần lượt từ LSB (bit 0) đến MSB (bit 7)
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end else begin
                        clk_cnt <= 16'd0;
                        if (bit_idx < 3'd7) begin
                            bit_idx <= bit_idx + 1'b1;
                        end else begin
                            bit_idx <= 3'd0;
                            state   <= ST_STOP;
                        end
                    end
                end

                ST_STOP: begin
                    tx_pin <= 1'b1; // Stop bit mức 1
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end else begin
                        clk_cnt <= 16'd0;
                        state   <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
