// =============================================================================
// File:        sensor_to_ascii.v
// Dự án:       Hệ thống Edge AI & Giám sát Cảm biến MPU6050 qua UART
// Mô tả:        Bộ chuyển đổi dữ liệu gia tốc và thông báo chẩn đoán sang UART ASCII.
//               Kiến trúc Zero-Latch (không dùng mảng RAM, 100% LUT logic thuần túy).
//               Tự động phát thông báo định kỳ (250ms) ngay cả khi không có cảm biến.
// =============================================================================

`timescale 1ns / 1ps

module sensor_to_ascii (
    input  wire                 clk,
    input  wire                 rst_n,

    // Dữ liệu từ mpu6050_hardware_reader
    input  wire                 data_valid,
    input  wire signed [15:0]   acc_x,
    input  wire signed [15:0]   acc_y,
    input  wire signed [15:0]   acc_z,
    input  wire                 device_ok,
    input  wire [7:0]           who_am_i,
    input  wire                 sda_stuck_low,

    // Giao tiếp với uart_tx
    output reg                  tx_start,
    output reg  [7:0]           tx_data,
    input  wire                 tx_busy
);

    // -------------------------------------------------------------------------
    // 1. CHUYỂN ĐỔI BCD VÀ HEX (DOUBLE-DABBLE)
    // -------------------------------------------------------------------------
    function [19:0] bin_to_bcd;
        input [15:0] bin;
        integer i;
        reg [19:0] bcd;
        begin
            bcd = 20'd0;
            for (i = 15; i >= 0; i = i - 1) begin
                if (bcd[3:0]   >= 5) bcd[3:0]   = bcd[3:0]   + 4'd3;
                if (bcd[7:4]   >= 5) bcd[7:4]   = bcd[7:4]   + 4'd3;
                if (bcd[11:8]  >= 5) bcd[11:8]  = bcd[11:8]  + 4'd3;
                if (bcd[15:12] >= 5) bcd[15:12] = bcd[15:12] + 4'd3;
                if (bcd[19:16] >= 5) bcd[19:16] = bcd[19:16] + 4'd3;
                bcd = {bcd[18:0], bin[i]};
            end
            bin_to_bcd = bcd;
        end
    endfunction

    function [7:0] to_hex_ascii;
        input [3:0] nibble;
        begin
            to_hex_ascii = (nibble < 4'd10) ? (8'h30 + nibble) : (8'h41 + (nibble - 4'd10));
        end
    endfunction

    // Chốt dữ liệu khi bắt đầu gửi gói để tránh jitter hiển thị
    reg        latched_sign_x, latched_sign_y, latched_sign_z;
    reg [19:0] latched_bcd_x,  latched_bcd_y,  latched_bcd_z;
    reg [7:0]  latched_id;
    reg [1:0]  latched_msg_type; // 0=BOOT, 1=SDA_ERR, 2=NO_ACK, 3=DATA_OK

    localparam [1:0]
        MSG_BOOT    = 2'd0,
        MSG_SDA_ERR = 2'd1,
        MSG_NO_ACK  = 2'd2,
        MSG_DATA_OK = 2'd3;

    // Bộ đếm ký tự và trạng thái
    reg [5:0]  char_idx;
    reg [5:0]  total_len;
    reg        transmitting;
    reg        boot_sent;
    reg [23:0] beacon_cnt;

    // Bộ tạo ký tự tổ hợp thuần túy (Zero-Latch ROM MUX)
    reg [7:0] char_out;
    always @(*) begin
        char_out = 8'h20; // Default: Space
        case (latched_msg_type)
            MSG_BOOT: begin
                // "\r\n=== FPGA MPU6050 & SD LOGGER ONLINE ===\r\n" (43 ký tự)
                case (char_idx)
                    6'd0:  char_out = 8'h0D;
                    6'd1:  char_out = 8'h0A;
                    6'd2:  char_out = "=";
                    6'd3:  char_out = "=";
                    6'd4:  char_out = "=";
                    6'd5:  char_out = " ";
                    6'd6:  char_out = "F";
                    6'd7:  char_out = "P";
                    6'd8:  char_out = "G";
                    6'd9:  char_out = "A";
                    6'd10: char_out = " ";
                    6'd11: char_out = "M";
                    6'd12: char_out = "P";
                    6'd13: char_out = "U";
                    6'd14: char_out = "6";
                    6'd15: char_out = "0";
                    6'd16: char_out = "5";
                    6'd17: char_out = "0";
                    6'd18: char_out = " ";
                    6'd19: char_out = "&";
                    6'd20: char_out = " ";
                    6'd21: char_out = "S";
                    6'd22: char_out = "D";
                    6'd23: char_out = " ";
                    6'd24: char_out = "O";
                    6'd25: char_out = "N";
                    6'd26: char_out = "L";
                    6'd27: char_out = "I";
                    6'd28: char_out = "N";
                    6'd29: char_out = "E";
                    6'd30: char_out = " ";
                    6'd31: char_out = "=";
                    6'd32: char_out = "=";
                    6'd33: char_out = "=";
                    6'd34: char_out = 8'h0D;
                    6'd35: char_out = 8'h0A;
                    default: char_out = 8'h20;
                endcase
            end

            MSG_SDA_ERR: begin
                // "MPU: [ERR: SDA KET 0V] Ktra Pin 40!\r\n" (37 ký tự)
                case (char_idx)
                    6'd0:  char_out = "M";
                    6'd1:  char_out = "P";
                    6'd2:  char_out = "U";
                    6'd3:  char_out = ":";
                    6'd4:  char_out = " ";
                    6'd5:  char_out = "[";
                    6'd6:  char_out = "E";
                    6'd7:  char_out = "R";
                    6'd8:  char_out = "R";
                    6'd9:  char_out = ":";
                    6'd10: char_out = " ";
                    6'd11: char_out = "S";
                    6'd12: char_out = "D";
                    6'd13: char_out = "A";
                    6'd14: char_out = " ";
                    6'd15: char_out = "K";
                    6'd16: char_out = "E";
                    6'd17: char_out = "T";
                    6'd18: char_out = " ";
                    6'd19: char_out = "0";
                    6'd20: char_out = "V";
                    6'd21: char_out = "]";
                    6'd22: char_out = " ";
                    6'd23: char_out = "P";
                    6'd24: char_out = "i";
                    6'd25: char_out = "n";
                    6'd26: char_out = " ";
                    6'd27: char_out = "4";
                    6'd28: char_out = "0";
                    6'd29: char_out = "!";
                    6'd30: char_out = 8'h0D;
                    6'd31: char_out = 8'h0A;
                    default: char_out = 8'h20;
                endcase
            end

            MSG_NO_ACK: begin
                // "MPU[ID=XX]: NO ACK! Ktra SCL(39) SDA(40) VCC\r\n" (46 ký tự)
                case (char_idx)
                    6'd0:  char_out = "M";
                    6'd1:  char_out = "P";
                    6'd2:  char_out = "U";
                    6'd3:  char_out = "[";
                    6'd4:  char_out = "I";
                    6'd5:  char_out = "D";
                    6'd6:  char_out = "=";
                    6'd7:  char_out = to_hex_ascii(latched_id[7:4]);
                    6'd8:  char_out = to_hex_ascii(latched_id[3:0]);
                    6'd9:  char_out = "]";
                    6'd10: char_out = ":";
                    6'd11: char_out = " ";
                    6'd12: char_out = "N";
                    6'd13: char_out = "O";
                    6'd14: char_out = " ";
                    6'd15: char_out = "A";
                    6'd16: char_out = "C";
                    6'd17: char_out = "K";
                    6'd18: char_out = "!";
                    6'd19: char_out = " ";
                    6'd20: char_out = "K";
                    6'd21: char_out = "t";
                    6'd22: char_out = "r";
                    6'd23: char_out = "a";
                    6'd24: char_out = " ";
                    6'd25: char_out = "P";
                    6'd26: char_out = "i";
                    6'd27: char_out = "n";
                    6'd28: char_out = " ";
                    6'd29: char_out = "3";
                    6'd30: char_out = "9";
                    6'd31: char_out = ",";
                    6'd32: char_out = "4";
                    6'd33: char_out = "0";
                    6'd34: char_out = 8'h0D;
                    6'd35: char_out = 8'h0A;
                    default: char_out = 8'h20;
                endcase
            end

            MSG_DATA_OK: begin
                // "MPU[68]: Ax=+00000 Ay=+00000 Az=+16384 | OK\r\n" (46 ký tự)
                case (char_idx)
                    6'd0:  char_out = "M";
                    6'd1:  char_out = "P";
                    6'd2:  char_out = "U";
                    6'd3:  char_out = "[";
                    6'd4:  char_out = to_hex_ascii(latched_id[7:4]);
                    6'd5:  char_out = to_hex_ascii(latched_id[3:0]);
                    6'd6:  char_out = "]";
                    6'd7:  char_out = ":";
                    6'd8:  char_out = " ";
                    6'd9:  char_out = "A";
                    6'd10: char_out = "x";
                    6'd11: char_out = "=";
                    6'd12: char_out = latched_sign_x ? "-" : "+";
                    6'd13: char_out = 8'h30 + latched_bcd_x[19:16];
                    6'd14: char_out = 8'h30 + latched_bcd_x[15:12];
                    6'd15: char_out = 8'h30 + latched_bcd_x[11:8];
                    6'd16: char_out = 8'h30 + latched_bcd_x[7:4];
                    6'd17: char_out = 8'h30 + latched_bcd_x[3:0];

                    6'd18: char_out = " ";
                    6'd19: char_out = "A";
                    6'd20: char_out = "y";
                    6'd21: char_out = "=";
                    6'd22: char_out = latched_sign_y ? "-" : "+";
                    6'd23: char_out = 8'h30 + latched_bcd_y[19:16];
                    6'd24: char_out = 8'h30 + latched_bcd_y[15:12];
                    6'd25: char_out = 8'h30 + latched_bcd_y[11:8];
                    6'd26: char_out = 8'h30 + latched_bcd_y[7:4];
                    6'd27: char_out = 8'h30 + latched_bcd_y[3:0];

                    6'd28: char_out = " ";
                    6'd29: char_out = "A";
                    6'd30: char_out = "z";
                    6'd31: char_out = "=";
                    6'd32: char_out = latched_sign_z ? "-" : "+";
                    6'd33: char_out = 8'h30 + latched_bcd_z[19:16];
                    6'd34: char_out = 8'h30 + latched_bcd_z[15:12];
                    6'd35: char_out = 8'h30 + latched_bcd_z[11:8];
                    6'd36: char_out = 8'h30 + latched_bcd_z[7:4];
                    6'd37: char_out = 8'h30 + latched_bcd_z[3:0];

                    6'd38: char_out = " ";
                    6'd39: char_out = "|";
                    6'd40: char_out = " ";
                    6'd41: char_out = "O";
                    6'd42: char_out = "K";
                    6'd43: char_out = 8'h0D;
                    6'd44: char_out = 8'h0A;
                    default: char_out = 8'h20;
                endcase
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // 2. MÁY TRẠNG THÁI PHÁT KÝ TỰ QUA UART
    // -------------------------------------------------------------------------
    wire        curr_sign_x = acc_x[15];
    wire [15:0] curr_mag_x  = curr_sign_x ? (-acc_x) : acc_x;

    wire        curr_sign_y = acc_y[15];
    wire [15:0] curr_mag_y  = curr_sign_y ? (-acc_y) : acc_y;

    wire        curr_sign_z = acc_z[15];
    wire [15:0] curr_mag_z  = curr_sign_z ? (-acc_z) : acc_z;

    // Chu kỳ beacon: 250ms tại 27 MHz = 6,750,000 chu kỳ
    localparam BEACON_PERIOD = 24'd6_750_000;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_start         <= 1'b0;
            tx_data          <= 8'd0;
            char_idx         <= 6'd0;
            total_len        <= 6'd0;
            transmitting     <= 1'b0;
            boot_sent        <= 1'b0;
            beacon_cnt       <= 24'd0;
            latched_msg_type <= MSG_BOOT;
            latched_sign_x   <= 1'b0;
            latched_sign_y   <= 1'b0;
            latched_sign_z   <= 1'b0;
            latched_bcd_x    <= 20'd0;
            latched_bcd_y    <= 20'd0;
            latched_bcd_z    <= 20'd0;
            latched_id       <= 8'd0;
        end else begin
            tx_start <= 1'b0;

            // Đếm chu kỳ gửi chẩn đoán định kỳ khi chưa nhận diện được MPU
            if (!device_ok) begin
                if (beacon_cnt < BEACON_PERIOD)
                    beacon_cnt <= beacon_cnt + 1'b1;
            end else begin
                beacon_cnt <= 24'd0;
            end

            // 1. Gửi Banner chào mừng ngay sau khi giải phóng reset
            if (!boot_sent && !transmitting) begin
                latched_msg_type <= MSG_BOOT;
                total_len        <= 6'd36;
                char_idx         <= 6'd0;
                transmitting     <= 1'b1;
                boot_sent        <= 1'b1;
            end
            // 2. Chốt dữ liệu cảm biến mới hoặc kích hoạt chẩn đoán định kỳ
            else if ((data_valid || (beacon_cnt >= BEACON_PERIOD)) && !transmitting) begin
                beacon_cnt <= 24'd0;
                latched_id <= who_am_i;

                if (sda_stuck_low) begin
                    latched_msg_type <= MSG_SDA_ERR;
                    total_len        <= 6'd32;
                end else if (!device_ok) begin
                    latched_msg_type <= MSG_NO_ACK;
                    total_len        <= 6'd36;
                end else begin
                    latched_msg_type <= MSG_DATA_OK;
                    latched_sign_x   <= curr_sign_x;
                    latched_sign_y   <= curr_sign_y;
                    latched_sign_z   <= curr_sign_z;
                    latched_bcd_x    <= bin_to_bcd(curr_mag_x);
                    latched_bcd_y    <= bin_to_bcd(curr_mag_y);
                    latched_bcd_z    <= bin_to_bcd(curr_mag_z);
                    total_len        <= 6'd45;
                end

                char_idx     <= 6'd0;
                transmitting <= 1'b1;
            end
            // 3. Truyền tuần tự từng byte sang uart_tx
            else if (transmitting) begin
                if (!tx_busy && !tx_start) begin
                    if (char_idx < total_len) begin
                        tx_data      <= char_out;
                        tx_start     <= 1'b1;
                        char_idx     <= char_idx + 1'b1;
                    end else begin
                        transmitting <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
