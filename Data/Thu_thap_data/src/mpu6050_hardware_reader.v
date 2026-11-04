// =============================================================================
// File:        mpu6050_hardware_reader.v
// Dự án:       Kiểm thử cảm biến MPU6050 & FPGA qua giao tiếp UART
// Mô tả:        Bộ điều khiển I2C Master phần cứng chuyên dụng đọc MPU6050:
//               - Tích hợp bộ lọc nhiễu số 3 tầng (Digital Glitch Filter) trên SDA/SCL.
//               - Tự động phát xung giải phóng Bus (I2C Bus Recovery 16 Clocks) khi khởi động.
//               - Tự động phát hiện đường SDA bị kẹt mức 0V (SDA Stuck-Low Detection).
//               - Tự động đánh thức cảm biến (Ghi 0x6B = 0x01: PLL Gyro X clock).
//               - Tự động kích hoạt toàn bộ 3 trục gia tốc (Ghi 0x6C = 0x00).
//               - Cấu hình bộ lọc thông thấp số DLPF 44 Hz (Ghi 0x1A = 0x03) triệt nhiễu rung lắc.
//               - Cấu hình dải đo nhạy nhất ±2g (Ghi 0x1C = 0x00: 1g = 16384 LSB).
//               - Đọc thanh ghi định danh WHO_AM_I (0x75) để chẩn đoán phần cứng.
//               - Quét liên tục 50 Hz đọc 6 byte gia tốc (0x3B..0x40).
// =============================================================================

`timescale 1ns / 1ps

module mpu6050_hardware_reader (
    input  wire                 clk,            // Xung nhịp hệ thống 50 MHz
    input  wire                 rst_n,          // Reset hệ thống (Active-Low)

    // Chân I2C vật lý nối trực tiếp ra cảm biến MPU6050
    inout  wire                 i2c_scl,        // SCL Push-Pull mạnh mẽ (Pin 39)
    inout  wire                 i2c_sda,        // SDA Open-Drain (Pin 40)

    // Dữ liệu gia tốc 3 trục đã đọc thành công (chuẩn Signed 16-bit)
    output reg  signed [15:0]   acc_x,
    output reg  signed [15:0]   acc_y,
    output reg  signed [15:0]   acc_z,
    output reg  [7:0]           who_am_i,       // Giá trị đọc từ WHO_AM_I (0x68)
    output reg                  data_valid,     // Xung 1 chu kỳ khi có gói dữ liệu mới

    // Tín hiệu chẩn đoán lỗi phần cứng
    output reg                  i2c_device_ok,  // 1: MPU6050 phản hồi ACK & WHO_AM_I hợp lệ
    output reg                  sda_stuck_low   // 1: Chân SDA bị kẹt 0V (chập mass hoặc thiếu trở kéo)
);

    // -------------------------------------------------------------------------
    // 1. GIAO DIỆN I2C VẬT LÝ & BỘ LỌC CHỐNG NHIỄU SỐ (DEGLITCH FILTER)
    // -------------------------------------------------------------------------
    reg scl_out_reg;
    reg sda_out_reg;

    assign i2c_scl = scl_out_reg;
    assign i2c_sda = (sda_out_reg) ? 1'bz : 1'b0;

    // Bộ lọc chống nhiễu số 3 tầng cho chân SDA (Majority Voting Filter)
    // Triệt tiêu hoàn toàn các gai xung nhiễu điện từ < 60ns trên dây cắm breadboard
    reg [2:0] sda_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sda_sync <= 3'b111;
        else
            sda_sync <= {sda_sync[1:0], i2c_sda};
    end
    wire sda_in = (sda_sync[2] & sda_sync[1]) | (sda_sync[1] & sda_sync[0]) | (sda_sync[2] & sda_sync[0]);

    // -------------------------------------------------------------------------
    // 2. BỘ PHÁT XUNG NHỊP I2C TỪ 50 MHz
    //    Tốc độ 50 kHz cho sườn xung an toàn trên mạch thật; Fast I2C khi mô phỏng
    // -------------------------------------------------------------------------
`ifdef SIMULATION
    localparam QUARTER_DIV   = 8'd10;         // Fast I2C trong mô phỏng (1.25 MHz)
    localparam POWER_DELAY   = 24'd100;       // 2 us delay cho mô phỏng
    localparam WAIT_INTERVAL = 24'd200;       // 4 us delay cho mô phỏng
`else
//    localparam QUARTER_DIV   = 8'd250;        // 50 MHz / (4 * 50 kHz) = 250
    localparam QUARTER_DIV   = 8'd31;  
    localparam POWER_DELAY   = 24'd2_500_000; // Chờ 50ms ổn định nguồn cảm biến
    localparam WAIT_INTERVAL = 24'd50_000; // Quét 
`endif
    reg [7:0]  q_cnt;
    reg        i2c_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_cnt    <= 8'd0;
            i2c_tick <= 1'b0;
        end else begin
            if (q_cnt >= QUARTER_DIV - 1) begin
                q_cnt    <= 8'd0;
                i2c_tick <= 1'b1;
            end else begin
                q_cnt    <= q_cnt + 1'b1;
                i2c_tick <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. FSM ĐIỀU KHIỂN GIAO THỨC I2C
    // -------------------------------------------------------------------------
    localparam [3:0]
        S_RECOVER     = 4'd0,  // Phát 16 clock giải phóng bus nếu slave bị kẹt
        S_POWER_DELAY = 4'd1,  // Chờ 50ms ổn định nguồn
        S_CHECK_BUS   = 4'd2,  // Kiểm tra chân SDA có kéo lên 3.3V không
        S_START       = 4'd3,  // Tạo điều kiện START chuẩn NXP
        S_SEND_BYTE   = 4'd4,  // Gửi 8-bit dữ liệu
        S_CHECK_ACK   = 4'd5,  // Kiểm tra ACK từ Slave
        S_READ_BYTE   = 4'd6,  // Đọc 8-bit từ Slave
        S_SEND_ACK    = 4'd7,  // Master gửi ACK (0) hoặc NACK (1)
        S_STOP        = 4'd8,  // Tạo điều kiện STOP chuẩn NXP
        S_WAIT_NEXT   = 4'd9;  // Chờ chu kỳ quét tiếp theo (20ms)

    reg [3:0]  fsm_state;
    reg [1:0]  sub_step;
    reg [23:0] timer_cnt;
    reg [4:0]  recover_cnt;

    reg [6:0]  dev_addr; // Tự động quét 0x68 và 0x69
    reg [7:0]  tx_byte;
    reg [7:0]  rx_byte;
    reg [2:0]  bit_cnt;
    reg        send_ack_val;
    reg        ack_error;

    reg [7:0]  data_b0, data_b1, data_b2, data_b3, data_b4, data_b5;
    reg [4:0]  seq_step;
    reg [3:0]  read_idx;

    // Các bước tuần tự (seq_step):
    // 0: Ghi 0x6B <= 0x01 (Wake-up chip, Clock Source = PLL Gyro X)
    // 1: Ghi 0x6C <= 0x00 (Kích hoạt toàn bộ 3 trục gia tốc & con quay)
    // 2: Ghi 0x1A <= 0x03 (Cấu hình DLPF 44 Hz triệt nhiễu rung)
    // 3: Ghi 0x1C <= 0x00 (Cấu hình dải đo Accel ±2g nhạy nhất: 16384 LSB/g)
    // 4: Đọc thanh ghi WHO_AM_I (0x75)
    // 5: Ghi con trỏ 0x3B (ACCEL_XOUT_H)
    // 6: Đọc liên tiếp 6 byte gia tốc (X_H, X_L, Y_H, Y_L, Z_H, Z_L)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state     <= S_RECOVER;
            sub_step      <= 2'd0;
            timer_cnt     <= 24'd0;
            recover_cnt   <= 5'd0;
            scl_out_reg   <= 1'b1;
            sda_out_reg   <= 1'b1;
            dev_addr      <= 7'h68;
            tx_byte       <= 8'd0;
            rx_byte       <= 8'd0;
            bit_cnt       <= 3'd0;
            send_ack_val  <= 1'b0;
            ack_error     <= 1'b0;
            seq_step      <= 5'd0;
            read_idx      <= 4'd0;
            acc_x         <= 16'sd0;
            acc_y         <= 16'sd0;
            acc_z         <= 16'sd0;
            who_am_i      <= 8'h00;
            data_valid    <= 1'b0;
            i2c_device_ok <= 1'b0;
            sda_stuck_low <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            case (fsm_state)
                // -------------------------------------------------------------
                // BƯỚC 0: PHÁT 16 XUNG CLOCK ĐỂ GIẢI PHÓNG BUS (I2C BUS RECOVERY)
                //         Đảm bảo nếu Slave bị kẹt do nạp lại FPGA thì sẽ nhả SDA
                // -------------------------------------------------------------
                S_RECOVER: begin
                    sda_out_reg <= 1'b1;
                    if (i2c_tick) begin
                        case (sub_step)
                            2'd0: begin scl_out_reg <= 1'b0; sub_step <= 2'd1; end
                            2'd1: begin scl_out_reg <= 1'b1; sub_step <= 2'd2; end
                            2'd2: begin scl_out_reg <= 1'b1; sub_step <= 2'd3; end
                            2'd3: begin
                                scl_out_reg <= 1'b0;
                                sub_step    <= 2'd0;
                                if (recover_cnt < 5'd16) begin
                                    recover_cnt <= recover_cnt + 1'b1;
                                end else begin
                                    scl_out_reg <= 1'b1;
                                    fsm_state   <= S_POWER_DELAY;
                                end
                            end
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // BƯỚC 1: CHỜ 50ms ỔN ĐỊNH NGUỒN CẢM BIẾN
                // -------------------------------------------------------------
                S_POWER_DELAY: begin
                    scl_out_reg <= 1'b1;
                    sda_out_reg <= 1'b1;
                    if (timer_cnt < POWER_DELAY) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt <= 24'd0;
                        fsm_state <= S_CHECK_BUS;
                    end
                end

                // -------------------------------------------------------------
                // BƯỚC 2: KIỂM TRA ĐƯỜNG SDA CÓ ĐƯỢC PULL-UP LÊN 3.3V KHÔNG
                // -------------------------------------------------------------
                S_CHECK_BUS: begin
                    scl_out_reg <= 1'b1;
                    sda_out_reg <= 1'b1;
                    if (sda_in == 1'b0) begin
                        // SDA bị kẹt ở mức 0V (Chập mass hoặc thiếu trở kéo 3.3V)
                        sda_stuck_low <= 1'b1;
                        i2c_device_ok <= 1'b0;
                        data_valid    <= 1'b1; // Xuất cảnh báo lên UART
                        fsm_state     <= S_WAIT_NEXT;
                    end else begin
                        sda_stuck_low <= 1'b0;
                        seq_step      <= 5'd0; // Bắt đầu chuỗi cấu hình
                        fsm_state     <= S_START;
                    end
                end

                // -------------------------------------------------------------
                // TẠO ĐIỀU KIỆN START CHUẨN I2C (SDA rơi từ 1 xuống 0 khi SCL đang 1)
                // -------------------------------------------------------------
                S_START: begin
                    if (i2c_tick) begin
                        case (sub_step)
                            2'd0: begin sda_out_reg <= 1'b1; scl_out_reg <= 1'b1; sub_step <= 2'd1; end
                            2'd1: begin sda_out_reg <= 1'b1; scl_out_reg <= 1'b1; sub_step <= 2'd2; end
                            2'd2: begin sda_out_reg <= 1'b0; scl_out_reg <= 1'b1; sub_step <= 2'd3; end // START condition!
                            2'd3: begin
                                sda_out_reg <= 1'b0;
                                scl_out_reg <= 1'b0;
                                sub_step    <= 2'd0;
                                bit_cnt     <= 3'd7;

                                // Xác định byte đầu tiên cần gửi (Địa chỉ Write hay Read)
                                if (seq_step == 5'd7 || seq_step == 5'd8)
                                    tx_byte <= {dev_addr, 1'b1}; // Gửi Slave Address + READ
                                else
                                    tx_byte <= {dev_addr, 1'b0}; // Gửi Slave Address + WRITE
                                fsm_state <= S_SEND_BYTE;
                            end
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // GỬI 1 BYTE DỮ LIỆU (8 BIT, MSB FIRST)
                // -------------------------------------------------------------
                S_SEND_BYTE: begin
                    if (i2c_tick) begin
                        case (sub_step)
                            2'd0: begin
                                scl_out_reg <= 1'b0;
                                sda_out_reg <= tx_byte[bit_cnt];
                                sub_step    <= 2'd1;
                            end
                            2'd1: begin scl_out_reg <= 1'b1; sub_step <= 2'd2; end
                            2'd2: begin scl_out_reg <= 1'b1; sub_step <= 2'd3; end
                            2'd3: begin
                                scl_out_reg <= 1'b0;
                                sub_step    <= 2'd0;
                                if (bit_cnt > 3'd0) begin
                                    bit_cnt <= bit_cnt - 1'b1;
                                end else begin
                                    fsm_state <= S_CHECK_ACK;
                                end
                            end
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // KIỂM TRA XUNG ACK TỪ SLAVE
                // -------------------------------------------------------------
                S_CHECK_ACK: begin
                    if (i2c_tick) begin
                        case (sub_step)
                            2'd0: begin
                                scl_out_reg <= 1'b0;
                                sda_out_reg <= 1'b1; // Nhả SDA để Slave kéo xuống 0 (ACK)
                                sub_step    <= 2'd1;
                            end
                            2'd1: begin scl_out_reg <= 1'b1; sub_step <= 2'd2; end
                            2'd2: begin
                                if (sda_in == 1'b0) begin
                                    ack_error <= 1'b0;
                                end else begin
                                    ack_error <= 1'b1;
                                end
                                sub_step <= 2'd3;
                            end
                            2'd3: begin
                                scl_out_reg <= 1'b0;
                                sub_step    <= 2'd0;
                                bit_cnt     <= 3'd7;

                                if (ack_error) begin
                                    // Bị lỗi NACK -> Quét đảo giữa 0x68 và 0x69 để tìm chip
                                    if (seq_step == 5'd0)
                                        dev_addr <= (dev_addr == 7'h68) ? 7'h69 : 7'h68;
                                    i2c_device_ok <= 1'b0;
                                    data_valid    <= 1'b1; // Báo lỗi lên UART
                                    fsm_state     <= S_STOP;
                                    seq_step      <= 5'd0; // Thử lại từ đầu
                                end else begin
                                    // Bắt tay ACK thành công -> Chuyển bước tùy theo chuỗi
                                    case (seq_step)
                                        // ---------------------------------------------
                                        // 0: Wake-up PWR_MGMT_1 (0x6B) <= 0x01
                                        // ---------------------------------------------
                                        5'd0: begin
                                            seq_step  <= 5'd10;
                                            tx_byte   <= 8'h6B; // Thanh ghi PWR_MGMT_1
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd10: begin
                                            seq_step  <= 5'd11;
                                            tx_byte   <= 8'h01; // Đánh thức chip, chọn clock PLL Gyro X
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd11: begin
                                            seq_step  <= 5'd1;  // Xong lệnh Wake-up -> sang kích hoạt trục
                                            fsm_state <= S_STOP;
                                        end

                                        // ---------------------------------------------
                                        // 1: Enable All Axes PWR_MGMT_2 (0x6C) <= 0x00
                                        // ---------------------------------------------
                                        5'd1: begin
                                            seq_step  <= 5'd12;
                                            tx_byte   <= 8'h6C; // Thanh ghi PWR_MGMT_2
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd12: begin
                                            seq_step  <= 5'd13;
                                            tx_byte   <= 8'h00; // Bật cả 3 trục Accel và 3 trục Gyro
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd13: begin
                                            seq_step  <= 5'd2;  // Xong -> sang cấu hình lọc DLPF
                                            fsm_state <= S_STOP;
                                        end

                                        // ---------------------------------------------
                                        // 2: Low-Pass Filter CONFIG (0x1A) <= 0x03 (44 Hz)
                                        // ---------------------------------------------
                                        5'd2: begin
                                            seq_step  <= 5'd14;
                                            tx_byte   <= 8'h1A; // Thanh ghi CONFIG
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd14: begin
                                            seq_step  <= 5'd15;
                                            tx_byte   <= 8'h03; // Lọc DLPF 44 Hz triệt nhiễu rung lắc
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd15: begin
                                            seq_step  <= 5'd3;  // Xong -> sang cấu hình dải đo
                                            fsm_state <= S_STOP;
                                        end

                                        // ---------------------------------------------
                                        // 3: Accel Scale ACCEL_CONFIG (0x1C) <= 0x00 (±2g)
                                        // ---------------------------------------------
                                        5'd3: begin
                                            seq_step  <= 5'd16;
                                            tx_byte   <= 8'h1C; // Thanh ghi ACCEL_CONFIG
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd16: begin
                                            seq_step  <= 5'd17;
                                            tx_byte   <= 8'h00; // Dải đo ±2g (Độ nhạy cao nhất: 16384 LSB/g)
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd17: begin
                                            seq_step  <= 5'd4;  // Xong cấu hình -> sang đọc WHO_AM_I
                                            fsm_state <= S_STOP;
                                        end

                                        // ---------------------------------------------
                                        // 4: Đọc thanh ghi WHO_AM_I (0x75)
                                        // ---------------------------------------------
                                        5'd4: begin
                                            seq_step  <= 5'd18;
                                            tx_byte   <= 8'h75; // Con trỏ thanh ghi WHO_AM_I
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd18: begin
                                            seq_step  <= 5'd7;  // Phát STOP rồi START đọc 1 byte
                                            fsm_state <= S_STOP;
                                        end
                                        5'd7: begin
                                            // Sau khi gửi địa chỉ Read (0xD1), chuyển sang nhận byte WHO_AM_I
                                            read_idx  <= 4'd0;
                                            fsm_state <= S_READ_BYTE;
                                        end

                                        // ---------------------------------------------
                                        // 5: Ghi con trỏ ACCEL_XOUT_H (0x3B)
                                        // ---------------------------------------------
                                        5'd5: begin
                                            seq_step  <= 5'd19;
                                            tx_byte   <= 8'h3B; // Con trỏ địa chỉ gia tốc
                                            fsm_state <= S_SEND_BYTE;
                                        end
                                        5'd19: begin
                                            seq_step  <= 5'd8;  // Phát STOP rồi START đọc liên tiếp 6 byte
                                            fsm_state <= S_STOP;
                                        end
                                        5'd8: begin
                                            // Sau khi gửi địa chỉ Read (0xD1), chuyển sang nhận byte gia tốc 0
                                            read_idx  <= 4'd0;
                                            fsm_state <= S_READ_BYTE;
                                        end

                                        default: fsm_state <= S_STOP;
                                    endcase
                                end
                            end
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // NHẬN 1 BYTE DỮ LIỆU TỪ SLAVE
                // -------------------------------------------------------------
                S_READ_BYTE: begin
                    if (i2c_tick) begin
                        case (sub_step)
                            2'd0: begin
                                scl_out_reg <= 1'b0;
                                sda_out_reg <= 1'b1; // Nhả SDA để Slave truyền
                                sub_step    <= 2'd1;
                            end
                            2'd1: begin scl_out_reg <= 1'b1; sub_step <= 2'd2; end
                            2'd2: begin
                                rx_byte[bit_cnt] <= sda_in; // Đọc bit từ sda_in đã qua bộ lọc chống nhiễu
                                sub_step         <= 2'd3;
                            end
                            2'd3: begin
                                scl_out_reg <= 1'b0;
                                sub_step    <= 2'd0;
                                if (bit_cnt > 3'd0) begin
                                    bit_cnt <= bit_cnt - 1'b1;
                                end else begin
                                    // Nếu là byte WHO_AM_I hoặc byte gia tốc cuối cùng (idx 5) thì Master gửi NACK (1)
                                    if (seq_step == 5'd7 || (seq_step == 5'd8 && read_idx == 4'd5))
                                        send_ack_val <= 1'b1; // NACK để kết thúc chuỗi đọc
                                    else
                                        send_ack_val <= 1'b0; // ACK để đọc tiếp
                                    fsm_state <= S_SEND_ACK;
                                end
                            end
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // MASTER GỬI XUNG ACK / NACK CHO SLAVE
                // -------------------------------------------------------------
                S_SEND_ACK: begin
                    if (i2c_tick) begin
                        case (sub_step)
                            2'd0: begin
                                scl_out_reg <= 1'b0;
                                sda_out_reg <= send_ack_val;
                                sub_step    <= 2'd1;
                            end
                            2'd1: begin scl_out_reg <= 1'b1; sub_step <= 2'd2; end
                            2'd2: begin sub_step <= 2'd3; end
                            2'd3: begin
                                scl_out_reg <= 1'b0;
                                sda_out_reg <= 1'b1;
                                sub_step    <= 2'd0;
                                bit_cnt     <= 3'd7;

                                if (seq_step == 5'd7) begin
                                    // Đã nhận xong byte WHO_AM_I
                                    who_am_i      <= rx_byte;
                                    i2c_device_ok <= (rx_byte == 8'h68 || rx_byte == 8'h70 || rx_byte == 8'h71 || rx_byte == 8'h72 || rx_byte == 8'h98);
                                    seq_step      <= 5'd5; // Chuyển sang đọc gia tốc
                                    fsm_state     <= S_STOP;
                                end else if (seq_step == 5'd8) begin
                                    // Lưu các byte gia tốc
                                    case (read_idx)
                                        4'd0: begin data_b0 <= rx_byte; read_idx <= 4'd1; fsm_state <= S_READ_BYTE; end
                                        4'd1: begin data_b1 <= rx_byte; read_idx <= 4'd2; fsm_state <= S_READ_BYTE; end
                                        4'd2: begin data_b2 <= rx_byte; read_idx <= 4'd3; fsm_state <= S_READ_BYTE; end
                                        4'd3: begin data_b3 <= rx_byte; read_idx <= 4'd4; fsm_state <= S_READ_BYTE; end
                                        4'd4: begin data_b4 <= rx_byte; read_idx <= 4'd5; fsm_state <= S_READ_BYTE; end
                                        4'd5: begin
                                            data_b5       <= rx_byte;
                                            acc_x         <= {data_b0, data_b1};
                                            acc_y         <= {data_b2, data_b3};
                                            acc_z         <= {data_b4, rx_byte};
                                            data_valid    <= 1'b1; // Báo có mẫu mới hoàn tất
                                            i2c_device_ok <= 1'b1;
                                            seq_step      <= 5'd5; // Chu kỳ sau tiếp tục đọc gia tốc
                                            fsm_state     <= S_STOP;
                                        end
                                        default: fsm_state <= S_STOP;
                                    endcase
                                end else begin
                                    fsm_state <= S_STOP;
                                end
                            end
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // TẠO ĐIỀU KIỆN STOP CHUẨN I2C (SDA từ 0 lên 1 khi SCL đang 1)
                // -------------------------------------------------------------
                S_STOP: begin
                    if (i2c_tick) begin
                        case (sub_step)
                            2'd0: begin sda_out_reg <= 1'b0; scl_out_reg <= 1'b0; sub_step <= 2'd1; end
                            2'd1: begin sda_out_reg <= 1'b0; scl_out_reg <= 1'b1; sub_step <= 2'd2; end
                            2'd2: begin sda_out_reg <= 1'b1; scl_out_reg <= 1'b1; sub_step <= 2'd3; end // STOP condition!
                            2'd3: begin
                                sub_step <= 2'd0;
                                // Nếu đang trong chuỗi khởi tạo (seq_step < 5) hoặc cần đọc ngay:
                                if (seq_step == 5'd1 || seq_step == 5'd2 || seq_step == 5'd3 || seq_step == 5'd4 || seq_step == 5'd7 || seq_step == 5'd8) begin
                                    fsm_state <= S_START;
                                end else begin
                                    fsm_state <= S_WAIT_NEXT;
                                end
                            end
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // CHỜ CHU KỲ QUÉT TIẾP THEO (Bình thường: 50 Hz = 20ms; Lỗi: 250ms)
                // -------------------------------------------------------------
                S_WAIT_NEXT: begin
                    scl_out_reg <= 1'b1;
                    sda_out_reg <= 1'b1;
                    if (timer_cnt < (i2c_device_ok ? WAIT_INTERVAL : (WAIT_INTERVAL * 4))) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt <= 24'd0;
                        if (i2c_device_ok) begin
                            seq_step  <= 5'd5; // Đọc gia tốc tiếp tục
                            fsm_state <= S_START;
                        end else begin
                            fsm_state <= S_CHECK_BUS; // Thử phát hiện lại từ đầu
                        end
                    end
                end

                default: fsm_state <= S_RECOVER;
            endcase
        end
    end

endmodule
