// =============================================================================
// File:        mpu6050_hardware_reader.v
// Dự án:       Bộ điều khiển I2C phần cứng đọc cảm biến MPU6050 (GY-521)
// Nền tảng:    Gowin GW1NSR-LV4CQN48PC6/I5 (Kit OneKiwi 4K / Tang Nano 4K)
// Phiên bản:   4.0 - Kiến trúc Command-Engine (Zero-Glitch, chuẩn công nghiệp)
// =============================================================================

`timescale 1ns / 1ps

module mpu6050_hardware_reader (
    input  wire                 clk,            // Xung nhịp hệ thống 50 MHz
    input  wire                 rst_n,          // Reset hệ thống (Active-Low)

    inout  wire                 i2c_scl,        // Pin 39 (hoặc 40 tùy scan_mode)
    inout  wire                 i2c_sda,        // Pin 40 (hoặc 39 tùy scan_mode)

    output reg  signed [15:0]   acc_x,
    output reg  signed [15:0]   acc_y,
    output reg  signed [15:0]   acc_z,
    output reg  [7:0]           who_am_i,
    output reg                  data_valid,

    output reg                  i2c_device_ok,
    output reg                  sda_stuck_low
);

    // =========================================================================
    // 1. QUÉT CHÂN VÀ ĐỊA CHỈ (AUTO-NEGOTIATION)
    // =========================================================================
    reg [1:0] scan_mode;
    wire [6:0] dev_addr = scan_mode[0] ? 7'h69 : 7'h68;

    reg scl_drv; // 1 = High-Z (pull-up), 0 = Pull low
    reg sda_drv;

    wire pin39_drv = scan_mode[1] ? sda_drv : scl_drv;
    wire pin40_drv = scan_mode[1] ? scl_drv : sda_drv;

    assign i2c_scl = pin39_drv ? 1'bz : 1'b0;
    assign i2c_sda = pin40_drv ? 1'bz : 1'b0;

    wire raw_sda = scan_mode[1] ? i2c_scl : i2c_sda;

    // Bộ lọc chống nhiễu số 3 tầng cho chân SDA
    reg [2:0] sda_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sda_sync <= 3'b111;
        else
            sda_sync <= {sda_sync[1:0], raw_sda};
    end
    wire sda_in = (sda_sync[2] & sda_sync[1]) |
                  (sda_sync[1] & sda_sync[0]) |
                  (sda_sync[2] & sda_sync[0]);

    // =========================================================================
    // 2. BỘ PHÁT XUNG I2C TICK (Tốc độ chuẩn 100 kHz, 4 ticks / bit)
    //    50 MHz / (4 * 125) = 100 kHz
    // =========================================================================
`ifdef SIMULATION
    localparam DIV_COUNT   = 8'd10;
    localparam POWER_DELAY = 24'd100;
    localparam WAIT_DELAY  = 24'd200;
`else
    localparam DIV_COUNT   = 8'd68;          // 27 MHz / (4 * 68) = 99.26 kHz I2C Standard Mode
    localparam POWER_DELAY = 24'd1_350_000;  // 50ms chờ nguồn MPU ổn định tại 27 MHz
    localparam WAIT_DELAY  = 24'd270_000;    // 10ms (100 Hz lấy mẫu gia tốc) tại 27 MHz
`endif

    reg [7:0] clk_cnt;
    reg       i2c_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt  <= 8'd0;
            i2c_tick <= 1'b0;
        end else begin
            if (clk_cnt >= DIV_COUNT - 1) begin
                clk_cnt  <= 8'd0;
                i2c_tick <= 1'b1;
            end else begin
                clk_cnt  <= clk_cnt + 1'b1;
                i2c_tick <= 1'b0;
            end
        end
    end

    // =========================================================================
    // 3. I2C LOW-LEVEL ENGINE (START, WRITE_BYTE, READ_BYTE, STOP)
    // =========================================================================
    localparam [2:0]
        E_IDLE  = 3'd0,
        E_START = 3'd1,
        E_WRITE = 3'd2,
        E_READ  = 3'd3,
        E_STOP  = 3'd4;

    reg [2:0] eng_state;
    reg [2:0] eng_phase;
    reg [3:0] eng_bit;
    reg [7:0] eng_tx_data;
    reg [7:0] eng_rx_data;
    reg       eng_ack_val;  // ACK to send during READ (0=ACK, 1=NACK)
    reg       eng_nack_rec; // NACK received from slave during WRITE
    reg       eng_done;     // Strobe when operation completes

    // High-Level commands to engine
    reg [2:0] cmd_req;      // E_IDLE, E_START, E_WRITE, E_READ, E_STOP
    reg       cmd_start;    // Pulse to trigger command

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            eng_state    <= E_IDLE;
            eng_phase    <= 3'd0;
            eng_bit      <= 4'd0;
            eng_rx_data  <= 8'd0;
            eng_nack_rec <= 1'b0;
            eng_done     <= 1'b0;
            scl_drv      <= 1'b1;
            sda_drv      <= 1'b1;
        end else begin
            eng_done <= 1'b0;

            if (eng_state == E_IDLE) begin
                if (cmd_start) begin
                    eng_state <= cmd_req;
                    eng_phase <= 3'd0;
                    eng_bit   <= 4'd7;
                end
            end else if (i2c_tick) begin
                case (eng_state)
                    // ---------------------------------------------------------
                    // START: SDA falls while SCL is High -> SCL falls
                    // ---------------------------------------------------------
                    E_START: begin
                        case (eng_phase)
                            3'd0: begin sda_drv <= 1'b1; scl_drv <= 1'b1; eng_phase <= 3'd1; end
                            3'd1: begin sda_drv <= 1'b1; scl_drv <= 1'b1; eng_phase <= 3'd2; end
                            3'd2: begin sda_drv <= 1'b0; scl_drv <= 1'b1; eng_phase <= 3'd3; end // START!
                            3'd3: begin
                                sda_drv   <= 1'b0;
                                scl_drv   <= 1'b0; // SCL low
                                eng_phase <= 3'd0;
                                eng_done  <= 1'b1;
                                eng_state <= E_IDLE;
                            end
                            default: eng_phase <= 3'd0;
                        endcase
                    end

                    // ---------------------------------------------------------
                    // WRITE 8 BITS + READ ACK (Leaves SCL LOW at end!)
                    // ---------------------------------------------------------
                    E_WRITE: begin
                        case (eng_phase)
                            3'd0: begin
                                scl_drv <= 1'b0;
                                sda_drv <= (eng_bit < 4'd8) ? eng_tx_data[eng_bit[2:0]] : 1'b1; // Bit or release for ACK
                                eng_phase <= 3'd1;
                            end
                            3'd1: begin
                                scl_drv   <= 1'b1; // SCL rises
                                eng_phase <= 3'd2;
                            end
                            3'd2: begin
                                // Sample ACK on 9th clock (eng_bit == 8)
                                if (eng_bit == 4'd8)
                                    eng_nack_rec <= sda_in;
                                eng_phase <= 3'd3;
                            end
                            3'd3: begin
                                scl_drv   <= 1'b0; // SCL falls
                                eng_phase <= 3'd0;
                                if (eng_bit > 4'd0) begin
                                    if (eng_bit == 4'd8) begin
                                        // Done sending 8 bits + ACK!
                                        eng_done  <= 1'b1;
                                        eng_state <= E_IDLE;
                                    end else begin
                                        eng_bit <= eng_bit - 1'b1;
                                    end
                                end else begin
                                    // Finished bit 0, next is ACK bit 8
                                    eng_bit <= 4'd8;
                                end
                            end
                            default: eng_phase <= 3'd0;
                        endcase
                    end

                    // ---------------------------------------------------------
                    // READ 8 BITS + SEND ACK/NACK (Leaves SCL LOW at end!)
                    // ---------------------------------------------------------
                    E_READ: begin
                        case (eng_phase)
                            3'd0: begin
                                scl_drv <= 1'b0;
                                sda_drv <= (eng_bit == 4'd8) ? eng_ack_val : 1'b1; // Send ACK or release to read
                                eng_phase <= 3'd1;
                            end
                            3'd1: begin
                                scl_drv   <= 1'b1; // SCL rises
                                eng_phase <= 3'd2;
                            end
                            3'd2: begin
                                // Sample data bit
                                if (eng_bit < 4'd8)
                                    eng_rx_data[eng_bit[2:0]] <= sda_in;
                                eng_phase <= 3'd3;
                            end
                            3'd3: begin
                                scl_drv   <= 1'b0; // SCL falls
                                eng_phase <= 3'd0;
                                if (eng_bit > 4'd0) begin
                                    if (eng_bit == 4'd8) begin
                                        // Done receiving 8 bits + sent ACK!
                                        eng_done  <= 1'b1;
                                        eng_state <= E_IDLE;
                                    end else begin
                                        eng_bit <= eng_bit - 1'b1;
                                    end
                                end else begin
                                    // Finished bit 0, next is ACK bit 8
                                    eng_bit <= 4'd8;
                                end
                            end
                            default: eng_phase <= 3'd0;
                        endcase
                    end

                    // ---------------------------------------------------------
                    // STOP: SCL rises while SDA=0 -> SDA rises
                    // ---------------------------------------------------------
                    E_STOP: begin
                        case (eng_phase)
                            3'd0: begin sda_drv <= 1'b0; scl_drv <= 1'b0; eng_phase <= 3'd1; end
                            3'd1: begin sda_drv <= 1'b0; scl_drv <= 1'b1; eng_phase <= 3'd2; end
                            3'd2: begin sda_drv <= 1'b1; scl_drv <= 1'b1; eng_phase <= 3'd3; end // STOP!
                            3'd3: begin
                                eng_phase <= 3'd0;
                                eng_done  <= 1'b1;
                                eng_state <= E_IDLE;
                            end
                            default: eng_phase <= 3'd0;
                        endcase
                    end

                    default: eng_state <= E_IDLE;
                endcase
            end
        end
    end

    // =========================================================================
    // 4. HIGH-LEVEL MPU-6050 TRANSACTION SEQUENCER
    // =========================================================================
    localparam [4:0]
        SEQ_POWER_ON    = 5'd0,   // Delay 50ms
        SEQ_START_PROBE = 5'd1,   // START
        SEQ_SEND_ADDR_W = 5'd2,   // Gửi dev_addr + W
        // Cấu hình thanh ghi 0x6B (Wake-up)
        SEQ_SEND_REG_6B = 5'd3,   // 0x6B
        SEQ_SEND_DAT_6B = 5'd4,   // 0x00
        SEQ_STOP_6B     = 5'd5,   // STOP
        // Cấu hình thanh ghi 0x1A (CONFIG DLPF 44Hz)
        SEQ_START_1A    = 5'd6,
        SEQ_ADDR_1A     = 5'd7,
        SEQ_REG_1A      = 5'd8,
        SEQ_DAT_1A      = 5'd9,
        SEQ_STOP_1A     = 5'd10,
        // Cấu hình thanh ghi 0x1C (ACCEL_CONFIG +-2g)
        SEQ_START_1C    = 5'd11,
        SEQ_ADDR_1C     = 5'd12,
        SEQ_REG_1C      = 5'd13,
        SEQ_DAT_1C      = 5'd14,
        SEQ_STOP_1C     = 5'd15,
        // Đọc WHO_AM_I (0x75)
        SEQ_START_ID    = 5'd16,
        SEQ_ADDR_ID_W   = 5'd17,
        SEQ_REG_ID      = 5'd18,
        SEQ_START_ID_R  = 5'd19,  // Repeated START
        SEQ_ADDR_ID_R   = 5'd20,  // dev_addr + R
        SEQ_READ_ID     = 5'd21,  // Read 1 byte (NACK)
        SEQ_STOP_ID     = 5'd22,  // STOP
        // Chu kỳ đọc 6 bytes gia tốc (0x3B..0x40)
        SEQ_START_ACC   = 5'd23,
        SEQ_ADDR_ACC_W  = 5'd24,
        SEQ_REG_ACC     = 5'd25,  // 0x3B
        SEQ_START_ACC_R = 5'd26,  // Repeated START
        SEQ_ADDR_ACC_R  = 5'd27,  // dev_addr + R
        SEQ_READ_ACC_0  = 5'd28,  // Ax H (ACK)
        SEQ_READ_ACC_1  = 5'd29,  // Ax L (ACK)
        SEQ_READ_ACC_2  = 5'd30,  // Ay H (ACK)
        SEQ_READ_ACC_3  = 5'd31,  // Ay L (ACK)
        // Dùng thêm bit mở rộng cho byte 4, 5
        SEQ_WAIT_NEXT   = 5'd31;

    reg [5:0] seq_step;
    reg [23:0] timer_cnt;
    reg [7:0] raw_b0, raw_b1, raw_b2, raw_b3, raw_b4, raw_b5;
    reg       probe_locked;

    // Cảnh báo UART mỗi 0.5s nếu chưa thấy MPU
    reg [24:0] warn_timer;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seq_step      <= 6'd0;
            timer_cnt     <= 24'd0;
            cmd_req       <= E_IDLE;
            cmd_start     <= 1'b0;
            eng_tx_data   <= 8'd0;
            eng_ack_val   <= 1'b0;
            scan_mode     <= 2'd0;
            probe_locked  <= 1'b0;
            who_am_i      <= 8'h00;
            acc_x         <= 16'sd0;
            acc_y         <= 16'sd0;
            acc_z         <= 16'sd0;
            data_valid    <= 1'b0;
            i2c_device_ok <= 1'b0;
            sda_stuck_low <= 1'b0;
            warn_timer    <= 25'd0;
        end else begin
            cmd_start  <= 1'b0;
            data_valid <= 1'b0;

            if (!i2c_device_ok) begin
                if (warn_timer >= 25'd6_750_000) begin // 0.25s tại 27 MHz
                    warn_timer <= 25'd0;
                    data_valid <= 1'b1;
                end else begin
                    warn_timer <= warn_timer + 1'b1;
                end
            end

            case (seq_step)
                // -------------------------------------------------------------
                // 0: Khởi động chờ nguồn
                // -------------------------------------------------------------
                6'd0: begin
                    if (timer_cnt < POWER_DELAY) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt <= 24'd0;
                        sda_stuck_low <= (sda_in == 1'b0);
                        // Bắt đầu chuỗi thăm dò
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd1;
                    end
                end

                // -------------------------------------------------------------
                // 1: START xong -> Gửi Slave Address + Write
                // -------------------------------------------------------------
                6'd1: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= {dev_addr, 1'b0};
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd2;
                    end
                end

                // -------------------------------------------------------------
                // 2: Kiểm tra ACK của Address byte
                // -------------------------------------------------------------
                6'd2: begin
                    if (eng_done) begin
                        if (eng_nack_rec) begin
                            // NACK: Không có thiết bị ở mode này -> Chuyển mode & STOP
                            if (!probe_locked)
                                scan_mode <= scan_mode + 1'b1;
                            cmd_req   <= E_STOP;
                            cmd_start <= 1'b1;
                            seq_step  <= 6'd44; // Chờ STOP hoàn tất
                        end else begin
                            // ACK thành công! Tiếp tục cấu hình 0x6B (PWR_MGMT_1)
                            cmd_req     <= E_WRITE;
                            eng_tx_data <= 8'h6B;
                            cmd_start   <= 1'b1;
                            seq_step    <= 6'd3;
                        end
                    end
                end

                // -------------------------------------------------------------
                // 3: Ghi 0x6B xong -> Ghi 0x00 (Wake up)
                // -------------------------------------------------------------
                6'd3: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= 8'h00;
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd4;
                    end
                end

                // -------------------------------------------------------------
                // 4: Ghi data 0x00 xong -> STOP
                // -------------------------------------------------------------
                6'd4: begin
                    if (eng_done) begin
                        cmd_req   <= E_STOP;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd5;
                    end
                end

                // -------------------------------------------------------------
                // 5..8: Cấu hình 0x1A (DLPF 44Hz)
                // -------------------------------------------------------------
                6'd5: begin
                    if (eng_done) begin
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd6;
                    end
                end
                6'd6: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= {dev_addr, 1'b0};
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd7;
                    end
                end
                6'd7: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= 8'h1A;
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd8;
                    end
                end
                6'd8: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= 8'h03; // DLPF 44Hz
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd9;
                    end
                end
                6'd9: begin
                    if (eng_done) begin
                        cmd_req   <= E_STOP;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd10;
                    end
                end

                // -------------------------------------------------------------
                // 10..14: Cấu hình 0x1C (Scale +-2g)
                // -------------------------------------------------------------
                6'd10: begin
                    if (eng_done) begin
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd11;
                    end
                end
                6'd11: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= {dev_addr, 1'b0};
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd12;
                    end
                end
                6'd12: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= 8'h1C;
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd13;
                    end
                end
                6'd13: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= 8'h00; // +-2g
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd14;
                    end
                end
                6'd14: begin
                    if (eng_done) begin
                        cmd_req   <= E_STOP;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd15;
                    end
                end

                // -------------------------------------------------------------
                // 15..21: Đọc WHO_AM_I (0x75)
                // -------------------------------------------------------------
                6'd15: begin
                    if (eng_done) begin
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd16;
                    end
                end
                6'd16: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= {dev_addr, 1'b0};
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd17;
                    end
                end
                6'd17: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= 8'h75; // WHO_AM_I register pointer
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd18;
                    end
                end
                6'd18: begin
                    if (eng_done) begin
                        // Repeated START để đổi sang đọc
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd19;
                    end
                end
                6'd19: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= {dev_addr, 1'b1}; // READ address
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd20;
                    end
                end
                6'd20: begin
                    if (eng_done) begin
                        cmd_req     <= E_READ;
                        eng_ack_val <= 1'b1; // NACK kết thúc chuỗi 1 byte
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd21;
                    end
                end
                6'd21: begin
                    if (eng_done) begin
                        who_am_i      <= eng_rx_data;
                        i2c_device_ok <= (eng_rx_data != 8'hFF && eng_rx_data != 8'h00);
                        probe_locked  <= 1'b1; // Khóa cấu hình scan_mode vĩnh viễn!
                        cmd_req       <= E_STOP;
                        cmd_start     <= 1'b1;
                        seq_step      <= 6'd22;
                    end
                end

                // -------------------------------------------------------------
                // 22..35: Đọc liên tục 6 bytes gia tốc (0x3B..0x40)
                // -------------------------------------------------------------
                6'd22: begin
                    if (eng_done) begin
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd23;
                    end
                end
                6'd23: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= {dev_addr, 1'b0};
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd24;
                    end
                end
                6'd24: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= 8'h3B; // ACCEL_XOUT_H pointer
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd25;
                    end
                end
                6'd25: begin
                    if (eng_done) begin
                        // Repeated START
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd26;
                    end
                end
                6'd26: begin
                    if (eng_done) begin
                        cmd_req     <= E_WRITE;
                        eng_tx_data <= {dev_addr, 1'b1}; // READ
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd27;
                    end
                end

                // Byte 0: Ax_H (ACK)
                6'd27: begin
                    if (eng_done) begin
                        cmd_req     <= E_READ;
                        eng_ack_val <= 1'b0; // ACK
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd28;
                    end
                end
                // Byte 1: Ax_L (ACK)
                6'd28: begin
                    if (eng_done) begin
                        raw_b0      <= eng_rx_data;
                        cmd_req     <= E_READ;
                        eng_ack_val <= 1'b0; // ACK
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd29;
                    end
                end
                // Byte 2: Ay_H (ACK)
                6'd29: begin
                    if (eng_done) begin
                        raw_b1      <= eng_rx_data;
                        cmd_req     <= E_READ;
                        eng_ack_val <= 1'b0; // ACK
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd30;
                    end
                end
                // Byte 3: Ay_L (ACK)
                6'd30: begin
                    if (eng_done) begin
                        raw_b2      <= eng_rx_data;
                        cmd_req     <= E_READ;
                        eng_ack_val <= 1'b0; // ACK
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd31;
                    end
                end
                // Byte 4: Az_H (ACK)
                6'd31: begin
                    if (eng_done) begin
                        raw_b3      <= eng_rx_data;
                        cmd_req     <= E_READ;
                        eng_ack_val <= 1'b0; // ACK
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd32;
                    end
                end
                // Byte 5: Az_L (NACK - kết thúc)
                6'd32: begin
                    if (eng_done) begin
                        raw_b4      <= eng_rx_data;
                        cmd_req     <= E_READ;
                        eng_ack_val <= 1'b1; // NACK
                        cmd_start   <= 1'b1;
                        seq_step    <= 6'd33;
                    end
                end
                // Lưu dữ liệu gia tốc và phát STOP
                6'd33: begin
                    if (eng_done) begin
                        raw_b5     <= eng_rx_data;
                        acc_x      <= {raw_b0, raw_b1};
                        acc_y      <= {raw_b2, raw_b3};
                        acc_z      <= {raw_b4, eng_rx_data};
                        data_valid <= 1'b1;
                        i2c_device_ok <= 1'b1;

                        cmd_req   <= E_STOP;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd34;
                    end
                end

                // -------------------------------------------------------------
                // 34: Chờ lệnh STOP hoàn tất trước khi đếm timer lấy mẫu
                // -------------------------------------------------------------
                6'd34: begin
                    if (eng_done) begin
                        timer_cnt <= 24'd0;
                        seq_step  <= 6'd35;
                    end
                end

                // 35: Đếm timer 10ms (100 Hz lấy mẫu)
                6'd35: begin
                    if (timer_cnt < WAIT_DELAY) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt <= 24'd0;
                        // Tiếp tục vòng lặp đọc gia tốc
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd23;
                    end
                end

                // -------------------------------------------------------------
                // 44: Chờ lệnh STOP hoàn tất sau NACK
                // -------------------------------------------------------------
                6'd44: begin
                    if (eng_done) begin
                        timer_cnt <= 24'd0;
                        seq_step  <= 6'd45;
                    end
                end

                // 45: Delay sau khi NACK để bus & cảm biến ổn định trước khi thử mode mới
                6'd45: begin
                    if (timer_cnt < (WAIT_DELAY * 2)) begin
                        timer_cnt <= timer_cnt + 1'b1;
                    end else begin
                        timer_cnt <= 24'd0;
                        // Thử lại từ đầu ở scan_mode mới
                        cmd_req   <= E_START;
                        cmd_start <= 1'b1;
                        seq_step  <= 6'd1;
                    end
                end

                default: seq_step <= 6'd0;
            endcase
        end
    end

endmodule
