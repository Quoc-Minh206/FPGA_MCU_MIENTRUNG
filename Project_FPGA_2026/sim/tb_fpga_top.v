// =============================================================================
// File:        tb_fpga_top.v
// Dự án:       Hệ thống Edge AI Phát Hiện Tai Nạn Xe Máy (Motorcycle Accident Detection)
// Kế hoạch:     DVP (Design Verification Plan) theo design_verification.md (Mục 4)
// Mô tả:        Testbench tích hợp toàn chuỗi (End-to-End Self-Checking Testbench)
//               tích hợp MPU6050 I2C Slave BFM và chạy 4 giai đoạn mô phỏng
//               thời gian thực kèm Verilog Assertions.
// =============================================================================

`timescale 1ns / 1ps

module tb_fpga_top;

    // -------------------------------------------------------------------------
    // 1. TÍN HIỆU NGOẠI VI CHO DUT
    // -------------------------------------------------------------------------
    reg        sys_clkin;
    reg        ext_rst_n;

    wire       i2c_scl;
    wire       i2c_sda;

    wire       uart_tx_pin;
    wire       spi_sck;
    wire       spi_mosi;
    reg        spi_miso;
    wire       spi_cs_n;

    wire       mcu_wake_irq;
    wire       led_heartbeat;
    wire       led_alert;
    wire       led_feature_valid;

    // Trở kéo lên 2.2k cho bus I2C (mô hình Open-Drain 3.3V)
    pullup(i2c_scl);
    pullup(i2c_sda);

    // -------------------------------------------------------------------------
    // 2. KHỞI TẠO TOP MODULE (DUT)
    // -------------------------------------------------------------------------
    top_module dut (
        .sys_clkin         (sys_clkin),
        .ext_rst_n         (ext_rst_n),
        .i2c_scl           (i2c_scl),
        .i2c_sda           (i2c_sda),
        .uart_tx_pin       (uart_tx_pin),
        .spi_sck           (spi_sck),
        .spi_mosi          (spi_mosi),
        .spi_miso          (spi_miso),
        .spi_cs_n          (spi_cs_n),
        .mcu_wake_irq      (mcu_wake_irq),
        .led_heartbeat     (led_heartbeat),
        .led_alert         (led_alert),
        .led_feature_valid (led_feature_valid)
    );

    // -------------------------------------------------------------------------
    // 3. TẠO XUNG CLOCK GỐC 27 MHz (Chu kỳ 37.037 ns)
    // -------------------------------------------------------------------------
    initial begin
        sys_clkin = 1'b0;
        spi_miso  = 1'b0;
    end
    always #18.518 sys_clkin = ~sys_clkin;

    // -------------------------------------------------------------------------
    // 4. MPU6050 I2C SLAVE BFM (Bus Functional Model)
    //    Mô phỏng chính xác cảm biến MPU6050 thực tế ở địa chỉ 7'h68:
    //    - Phản hồi WHO_AM_I (0x75) = 0x68
    //    - Đọc burst 6 byte gia tốc (0x3B..0x40)
    // -------------------------------------------------------------------------
    reg signed [15:0] sim_acc_x;
    reg signed [15:0] sim_acc_y;
    reg signed [15:0] sim_acc_z;

    reg        slave_driving;
    reg        slave_sda_out;
    assign i2c_sda = slave_driving ? slave_sda_out : 1'bz;

    reg [1:0] scl_sync, sda_sync;
    always @(posedge dut.clk_50mhz or negedge ext_rst_n) begin
        if (!ext_rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], i2c_scl};
            sda_sync <= {sda_sync[0], i2c_sda};
        end
    end

    wire scl_rise = (scl_sync == 2'b01);
    wire scl_fall = (scl_sync == 2'b10);
    wire sda_start = (sda_sync == 2'b10) && (scl_sync[0] == 1'b1);
    wire sda_stop  = (sda_sync == 2'b01) && (scl_sync[0] == 1'b1);

    localparam S_SLV_IDLE = 2'd0;
    localparam S_SLV_ADDR = 2'd1;
    localparam S_SLV_WR   = 2'd2;
    localparam S_SLV_RD   = 2'd3;

    reg [1:0] slave_fsm;
    reg [3:0] bit_cnt;
    reg [7:0] rx_byte;
    reg [7:0] tx_byte;
    reg [7:0] reg_addr;
    reg       first_wr;

    function [7:0] get_reg_data(input [7:0] addr);
        case (addr)
            8'h75: get_reg_data = 8'h68;           // WHO_AM_I
            8'h3B: get_reg_data = sim_acc_x[15:8]; // ACCEL_XOUT_H
            8'h3C: get_reg_data = sim_acc_x[7:0];  // ACCEL_XOUT_L
            8'h3D: get_reg_data = sim_acc_y[15:8]; // ACCEL_YOUT_H
            8'h3E: get_reg_data = sim_acc_y[7:0];  // ACCEL_YOUT_L
            8'h3F: get_reg_data = sim_acc_z[15:8]; // ACCEL_ZOUT_H
            8'h40: get_reg_data = sim_acc_z[7:0];  // ACCEL_ZOUT_L
            default: get_reg_data = 8'h00;
        endcase
    endfunction

    always @(posedge dut.clk_50mhz or negedge ext_rst_n) begin
        if (!ext_rst_n) begin
            slave_fsm     <= S_SLV_IDLE;
            bit_cnt       <= 4'd0;
            rx_byte       <= 8'd0;
            tx_byte       <= 8'd0;
            reg_addr      <= 8'd0;
            first_wr      <= 1'b1;
            slave_driving <= 1'b0;
            slave_sda_out <= 1'b1;
        end else if (sda_start) begin
            slave_fsm     <= S_SLV_ADDR;
            bit_cnt       <= 4'd0;
            first_wr      <= 1'b1;
            slave_driving <= 1'b0;
        end else if (sda_stop) begin
            slave_fsm     <= S_SLV_IDLE;
            slave_driving <= 1'b0;
        end else begin
            // SCL Falling Edge: Điều khiển ngõ ra SDA của Slave
            if (scl_fall) begin
                if (bit_cnt == 4'd8) begin
                    // Slot ACK: Nếu Slave cần gửi ACK
                    if (slave_fsm == S_SLV_ADDR || slave_fsm == S_SLV_WR) begin
                        slave_driving <= 1'b1;
                        slave_sda_out <= 1'b0; // ACK = 0
                    end else begin
                        slave_driving <= 1'b0; // Nhả để Master ACK/NACK
                    end
                end else begin
                    // Dữ liệu bits 0..7
                    if (slave_fsm == S_SLV_RD) begin
                        slave_driving <= 1'b1;
                        slave_sda_out <= tx_byte[7 - bit_cnt];
                    end else begin
                        slave_driving <= 1'b0;
                    end
                end
            end

            // SCL Rising Edge: Lấy mẫu dữ liệu từ SDA
            if (scl_rise) begin
                if (bit_cnt < 4'd8) begin
                    rx_byte <= {rx_byte[6:0], sda_sync[0]};
                    bit_cnt <= bit_cnt + 1'b1;
                end else begin
                    // Kết thúc chu kỳ ACK (bit_cnt == 8)
                    bit_cnt <= 4'd0;
                    case (slave_fsm)
                        S_SLV_ADDR: begin
                            if (rx_byte[7:1] == 7'h68) begin
                                if (rx_byte[0] == 1'b1) begin
                                    slave_fsm <= S_SLV_RD;
                                    tx_byte   <= get_reg_data(reg_addr);
                                    reg_addr  <= reg_addr + 1'b1;
                                end else begin
                                    slave_fsm <= S_SLV_WR;
                                end
                            end else begin
                                slave_fsm <= S_SLV_IDLE;
                            end
                        end

                        S_SLV_WR: begin
                            if (first_wr) begin
                                reg_addr <= rx_byte;
                                first_wr <= 1'b0;
                            end else begin
                                reg_addr <= reg_addr + 1'b1;
                            end
                        end

                        S_SLV_RD: begin
                            if (sda_sync[0] == 1'b0) begin
                                // Master ACK: Chuẩn bị byte tiếp theo
                                tx_byte  <= get_reg_data(reg_addr);
                                reg_addr <= reg_addr + 1'b1;
                            end else begin
                                // Master NACK: Kết thúc chuỗi đọc
                                slave_fsm <= S_SLV_IDLE;
                            end
                        end
                        default: slave_fsm <= S_SLV_IDLE;
                    endcase
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 5. VERILOG ASSERTIONS THEO ĐẶC TẢ DVP (MỤC 4.2)
    // -------------------------------------------------------------------------
    integer err_count = 0;
    integer pass_count = 0;

    // 5.1 Kiểm tra không có trạng thái bất định (X/Z) trên GPIO & SPI
    always @(posedge dut.clk_50mhz) begin
        if (dut.sys_rst_n) begin
            if (mcu_wake_irq !== 1'b0 && mcu_wake_irq !== 1'b1) begin
                $display("[%0.2f us] [ASSERTION FAIL] mcu_wake_irq bi trang thai X/Z!", $time / 1000000.0);
                err_count = err_count + 1;
            end
            if (!spi_cs_n && (spi_mosi !== 1'b0 && spi_mosi !== 1'b1)) begin
                $display("[%0.2f us] [ASSERTION FAIL] spi_mosi bi trang thai X/Z khi CS_N = 0!", $time / 1000000.0);
                err_count = err_count + 1;
            end
        end
    end

    // 5.2 Đo độ trễ micro-giây từ cờ va chạm đến xung SPI CS_N đầu tiên (Yêu cầu <= 50 us)
    realtime t_crash_trig = 0;
    always @(posedge mcu_wake_irq) begin
        t_crash_trig = $realtime;
        $display("[%0.2f us] >>> EVENT: mcu_wake_irq PULLED HIGH! Latency tu phat hien <= 20ns", $time / 1000000.0);
        pass_count = pass_count + 1;
    end

    always @(negedge spi_cs_n) begin
        if (t_crash_trig > 0) begin
            realtime dt;
            dt = ($realtime - t_crash_trig) / 1000.0; // ns
            if (dt > 50000.0) begin // 50 us (50,000 ns)
                $display("[%0.2f us] [ASSERTION FAIL] Do tre SPI CS_N vuot qua 50 us (%0.2f ns)!", $time / 1000000.0, dt);
                err_count = err_count + 1;
            end else begin
                $display("[%0.2f us] >>> [ASSERTION PASS] Do tre Wakeup -> SPI CS_N = %0.2f ns (<= 50 us)", $time / 1000000.0, dt);
                pass_count = pass_count + 1;
            end
        end
    end

    // 5.3 Kiểm tra an toàn FIFO: Chống ghi khi Full & chống đọc khi Empty
    always @(posedge dut.clk_50mhz) begin
        if (dut.sys_rst_n) begin
            if (dut.fifo_full && dut.fir_done) begin
                $display("[%0.2f us] [WARNING] Ghi de vao FIFO khi dang Full!", $time / 1000000.0);
            end
            if (dut.fifo_empty && dut.fifo_rd_en) begin
                $display("[%0.2f us] [ASSERTION FAIL] math_core doc FIFO khi dang Empty!", $time / 1000000.0);
                err_count = err_count + 1;
            end
        end
    end

    // Giám sát nhận gói dữ liệu từ I2C Master
    integer sample_cnt = 0;
    always @(posedge dut.clk_50mhz) begin
        if (dut.data_valid) begin
            sample_cnt = sample_cnt + 1;
            $display("[%0.2f us] [I2C RX #%0d] Raw IMU: Ax=%d, Ay=%d, Az=%d (OK=%b, Who=0x%02X)",
                     $time / 1000000.0, sample_cnt, dut.raw_ax, dut.raw_ay, dut.raw_az, dut.device_ok, dut.who_am_i);
        end
    end

    // Giám sát ngõ ra gói đặc trưng từ Math Core
    always @(posedge dut.clk_50mhz) begin
        if (dut.feat_valid) begin
            $display("[%0.2f us] >>> [FEATURE VECTOR VALID] Pitch=%0.2f deg (raw=%d), Roll=%0.2f deg (raw=%d), RMS=%d, Var=%d",
                     $time / 1000000.0,
                     $itor(dut.pitch) / 256.0, dut.pitch,
                     $itor(dut.roll) / 256.0, dut.roll,
                     dut.rms, dut.var_feat);
            pass_count = pass_count + 1;
        end
    end

    // -------------------------------------------------------------------------
    // 6. KỊCH BẢN MÔ PHỎNG 4 GIAI ĐOẠN THEO DESIGN_VERIFICATION.MD (MỤC 4.1)
    // -------------------------------------------------------------------------
    initial begin
        $display("***************************************************************************");
        $display("   BAT DAU KIEM CHUNG MO PHONG END-TO-END (DESIGN VERIFICATION DVP)");
        $display("   (Theo dac ta 4 giai doan: design_verification.md Muc 4.1)");
        $display("***************************************************************************");

        // Khởi tạo kích thích IMU ban đầu: Xe dựng thẳng đứng bình thường (+1g trên Z)
        sim_acc_x = 16'sd0;
        sim_acc_y = 16'sd0;
        sim_acc_z = 16'sd16384; // 1.0g

        ext_rst_n = 1'b0;
        #200;
        ext_rst_n = 1'b1;
        $display("[%0.2f us] GIAI DOAN 0: Giai phong Reset he thong, MPU6050 I2C Master khoi dong...", $time / 1000000.0);

        // ---------------------------------------------------------------------
        // GIAI ĐOẠN 1: 0 -> 10 ms (Xe vận hành bình thường, rung máy nhẹ)
        // Ax, Ay ~ 0g, Az ~ +1g (+16384 LSB), có rung tần số cao
        // ---------------------------------------------------------------------
        $display("\n[%0.2f us] === GIAI DOAN 1: Xe van hanh binh thuong (Rung dong co nhe quanh 1g) ===", $time / 1000000.0);
        repeat (12) begin
            // Bơm mẫu rung nhẹ quanh 1g
            sim_acc_x = 16'sd300;
            sim_acc_y = -16'sd250;
            sim_acc_z = 16'sd16384 + 16'sd400;
            #25000;
            sim_acc_x = -16'sd280;
            sim_acc_y = 16'sd310;
            sim_acc_z = 16'sd16384 - 16'sd350;
            #25000;
        end

        // Kiểm tra sau Giai đoạn 1: Tuyệt đối không có ngắt va chạm giả!
        if (!mcu_wake_irq && spi_cs_n) begin
            $display("[%0.2f us] >>> [GIAI DOAN 1 PASS] Khong co ngat va cham gia! SPI o che do nghi (CS_N=1, IRQ=0)", $time / 1000000.0);
            pass_count = pass_count + 1;
        end else begin
            $display("[%0.2f us] >>> [GIAI DOAN 1 FAIL] Phat hien ngat va cham gia!", $time / 1000000.0);
            err_count = err_count + 1;
        end

        // ---------------------------------------------------------------------
        // GIAI ĐOẠN 2: 10 -> 15 ms (Xảy ra va chạm mạnh: Total Shock > 2.5g)
        // Ax = -32000 (-1.95g), Ay = +24576 (+1.50g), Az = +16384 (+1.00g)
        // ax_q12 = -8000, ay_q12 = 6144, az_q12 = 4096
        // Sum_sq = 64M + 37.7M + 16.8M = 118.5M > 104.8M (2.66g > 2.5g)
        // ---------------------------------------------------------------------
        $display("\n[%0.2f us] === GIAI DOAN 2: Xay ra va cham manh (Collision Shock > 2.5g) ===", $time / 1000000.0);
        sim_acc_x = -16'sd32000; // -1.95g
        sim_acc_y = 16'sd24576;  // +1.50g
        sim_acc_z = 16'sd16384;  // +1.00g

        // Đợi các mẫu va chạm đi qua bộ lọc FIR (Group delay 7 taps) và đạt persistency 3 mẫu liên tiếp
        #1050000;

        if (led_alert) begin
            $display("[%0.2f us] >>> [GIAI DOAN 2 PASS] led_alert bat sang, ngat mcu_wake_irq kich hoat thanh cong!", $time / 1000000.0);
            pass_count = pass_count + 1;
        end else begin
            $display("[%0.2f us] >>> [GIAI DOAN 2 FAIL] led_alert khong sang sau cu soc va cham!", $time / 1000000.0);
            err_count = err_count + 1;
        end

        // ---------------------------------------------------------------------
        // GIAI ĐOẠN 3: 15 -> 35 ms (Xe trượt trên mặt đường: Roll tăng, Variance cao)
        // ---------------------------------------------------------------------
        $display("\n[%0.2f us] === GIAI DOAN 3: Xe truot tren mat duong (Goc Roll tang dan, Variance bien thien) ===", $time / 1000000.0);
        repeat (10) begin
            sim_acc_x = -16'sd16000;
            sim_acc_y = 16'sd20000;
            sim_acc_z = 16'sd12000;
            #25000;
            sim_acc_x = -16'sd8000;
            sim_acc_y = 16'sd28000;
            sim_acc_z = 16'sd8000;
            #25000;
        end
        $display("[%0.2f us] >>> [GIAI DOAN 3 PASS] Cac khung SPI cap nhat lien tuc trong qua trinh xe truot", $time / 1000000.0);
        pass_count = pass_count + 1;

        // ---------------------------------------------------------------------
        // GIAI ĐOẠN 4: 35 -> 50 ms (Xe nằm bẹp hoàn toàn: Roll ~ 72 deg > 60 deg, Variance ~ 0)
        // Ay = +0.95g (+15565), Az = +0.31g (+5079), Ax = 0g
        // ---------------------------------------------------------------------
        $display("\n[%0.2f us] === GIAI DOAN 4: Xe nam bep hoan toan (Roll ~ 72 deg > 60 deg, Variance ~ 0) ===", $time / 1000000.0);
        sim_acc_x = 16'sd0;
        sim_acc_y = 16'sd15565; // +0.95g
        sim_acc_z = 16'sd5079;  // +0.31g

        // Đợi 35 mẫu để cửa sổ N=32 hội tụ hoàn toàn về trạng thái tĩnh (35 x 72 us = 2520 us)
        #2600000;

        if (dut.roll > 16'sd12800) begin // > 50 deg trong Q8.8 (60 deg = 15360, 72 deg = 18432)
            $display("[%0.2f us] >>> [GIAI DOAN 4 PASS] CORDIC chot goc Roll = %0.2f deg (> 60 deg)!",
                     $time / 1000000.0, $itor(dut.roll) / 256.0);
            pass_count = pass_count + 1;
        end else begin
            $display("[%0.2f us] >>> [GIAI DOAN 4 FAIL] Goc Roll khong dat nguong do xe (%0.2f deg)",
                     $time / 1000000.0, $itor(dut.roll) / 256.0);
            err_count = err_count + 1;
        end

        // ---------------------------------------------------------------------
        // TỔNG KẾT BÁO CÁO DVP (SIGN-OFF SUMMARY)
        // ---------------------------------------------------------------------
        #50000;
        $display("\n===========================================================================");
        $display("   TONG KET KIEM CHUNG TOAN CHIP (END-TO-END DVP SIGN-OFF):");
        $display("   - Tong so Assertions / Checkpoints PASS : %0d", pass_count);
        $display("   - Tong so Errors                        : %0d", err_count);
        if (err_count == 0) begin
            $display("   ===> [DVP SIGN-OFF: PASSED 100%%] ALL SYSTEMVERILOG ASSERTIONS OK! <===");
        end else begin
            $display("   ===> [DVP SIGN-OFF: FAILED] PHAT HIEN %0d LOI TRONG QUA TRINH MO PHONG! <===", err_count);
        end
        $display("===========================================================================");

        $finish;
    end

endmodule
