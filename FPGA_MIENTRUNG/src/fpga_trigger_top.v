// =============================================================================
// File:        fpga_trigger_top.v
// Project:     FPGA_MIENTRUNG - Edge AI Motorcycle Accident Detection System
// Target:      Gowin GW1NSR-LV4CQN48PC6/I5 (OneKiwi 4K / Tang Nano 4K)
// Purpose:     1. Heartbeat LED on Pin 10 (1 Hz).
//              2. Trigger WAKE IRQ on Pin 41 (Day DO -> MCU J7 Pin 4 / P2.9):
//                 - Khi bấm nút S1 (Pin 14) HOẶC S2 (Pin 15) trên bo FPGA.
//                 - Tự động phát xung chu kỳ mỗi 15 giây (Auto-trigger).
//                 - LED2 (Pin 13) sáng lên mỗi khi xung kích hoạt Active.
//              3. SPI Lines sang MCU Sonix J12:
//                 - Pin 44 (Xanh bien): SCK -> P1.0 MCU J12 Pin 1
//                 - Pin 43 (Tim):       MISO <- P1.1 MCU J12 Pin 2
//                 - Pin 42 (Xanh la):   MOSI -> P1.2 MCU J12 Pin 3
//                 - Pin 46 (Trang):     CS_N -> P1.8 MCU J12 Pin 4
// =============================================================================

`timescale 1ns / 1ps

module fpga_trigger_top (
    input  wire sys_clkin,          // 27 MHz onboard crystal (Pin 45)
    input  wire btn_s1,             // Nút S1 onboard (Pin 14, Active-Low)
    input  wire btn_s2,             // Nút S2 onboard (Pin 15, Active-Low)

    // Diagnostic LEDs onboard OneKiwi 4K
    output wire led_onboard_1,      // Pin 10: 1 Hz heartbeat LED
    output wire led_onboard_2,      // Pin 13: Trigger Active LED (HIGH = Active)

    // Trigger signals to MCU Sonix SN32F407 (Active-HIGH pulse)
    // Chân Pin 41 (Dây ĐỎ) nối thẳng sang chân P2.9 (Cụm J7 Pin 4) của MCU Sonix
    output wire trigger_p41,        // Pin 41: WAKE IRQ (Dây Đỏ nối sang P2.9 MCU)
    output wire trigger_p32,        // Pin 32
    output wire trigger_p31,        // Pin 31
    output wire trigger_p27,        // Pin 27
    output wire trigger_p28,        // Pin 28

    // SPI lines to MCU Sonix SN32F407 (Cụm J12)
    output wire spi_sck,            // Pin 44: SCK (Dây Xanh biển nối sang P1.0 MCU)
    input  wire spi_miso,           // Pin 43: MISO (Dây Tím nối từ P1.1 MCU về)
    output wire spi_mosi,           // Pin 42: MOSI (Dây Xanh lá nối sang P1.2 MCU)
    output wire spi_cs_n,           // Pin 46: CS_N (Dây Trắng nối sang P1.8 MCU)

    // I2C lines to MPU6050 (BẢNG 2 file constrain.pdf)
    output wire i2c_scl,            // Pin 39: SCL
    inout  wire i2c_sda             // Pin 40: SDA
);

    assign i2c_scl = 1'b1;          // Idle high
    assign i2c_sda = 1'bz;          // High-Z

    // -------------------------------------------------------------------------
    // 1. Debounce and synchronize S1 & S2 (Active-Low)
    // -------------------------------------------------------------------------
    reg [1:0] s1_sync = 2'b11;
    reg [1:0] s2_sync = 2'b11;
    always @(posedge sys_clkin) begin
        s1_sync <= {s1_sync[0], btn_s1};
        s2_sync <= {s2_sync[0], btn_s2};
    end
    wire button_pressed = (s1_sync[1] == 1'b0) || (s2_sync[1] == 1'b0); // S1 hoac S2 duoc bam

    // -------------------------------------------------------------------------
    // 2. Heartbeat Counter (27 MHz -> 1 Hz, toggle at 13,500,000 cycles)
    // -------------------------------------------------------------------------
    reg [24:0] hb_cnt = 25'd0;
    reg        hb_led = 1'b0;

    always @(posedge sys_clkin) begin
        if (hb_cnt >= 25'd13_499_999) begin
            hb_cnt <= 25'd0;
            hb_led <= ~hb_led;
        end else begin
            hb_cnt <= hb_cnt + 1'b1;
        end
    end

    assign led_onboard_1 = hb_led;

    // -------------------------------------------------------------------------
    // 3. Trigger Generator:
    //    - Thủ công: bấm nút S1 (Pin 14) hoặc S2 (Pin 15) trên bo FPGA
    //    - Độ rộng xung kích hoạt: 500 ms (13,500,000 chu kỳ @ 27 MHz)
    //    - Tuyệt đối không tự động phát xung định kỳ để tránh kích hoạt giả
    // -------------------------------------------------------------------------
    localparam [31:0] PULSE_CYCLES  = 32'd13_500_000;  // 500 ms

    reg [31:0] pulse_cnt = 32'd0;
    reg        trigger_active = 1'b0;
    reg        btn_prev = 1'b0;
    reg [23:0] power_on_lock = 24'd13_500_000; // Khóa 500ms khi vừa cắm điện để chống sốc nguồn

    always @(posedge sys_clkin) begin
        if (power_on_lock > 24'd0) begin
            power_on_lock <= power_on_lock - 1'b1;
            btn_prev <= 1'b0;
            trigger_active <= 1'b0;
        end else begin
            btn_prev <= button_pressed;

            // Button edge detection (Chỉ kích hoạt khi bấm nút thật sự)
            if (button_pressed && !btn_prev) begin
                trigger_active <= 1'b1;
                pulse_cnt <= PULSE_CYCLES;
            end

            // Pulse duration handler
            if (trigger_active) begin
                if (pulse_cnt > 32'd0) begin
                    pulse_cnt <= pulse_cnt - 1'b1;
                end else begin
                    trigger_active <= 1'b0;
                end
            end
        end
    end

    // LED2 sáng lên khi kích hoạt
    assign led_onboard_2 = trigger_active;

    // WAKE IRQ (Dây ĐỎ Pin 41 sang MCU P2.9)
    assign trigger_p41 = trigger_active;

    // Output trigger pulse to other pins
    assign trigger_p32 = trigger_active;
    assign trigger_p31 = trigger_active;
    assign trigger_p27 = trigger_active;
    assign trigger_p28 = trigger_active;

    // SPI lines sang MCU Sonix J12
    assign spi_sck     = trigger_active;  // Pin 44 (Dây Xanh biển -> P1.0 MCU)
    assign spi_mosi    = trigger_active;  // Pin 42 (Dây Xanh lá -> P1.2 MCU)
    assign spi_cs_n    = ~trigger_active; // Pin 46 (Dây Trắng -> P1.8 MCU)

endmodule
