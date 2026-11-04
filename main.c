/******************************************************************************
 * main.c  -  Hệ thống phát hiện tai nạn xe máy TinyML (SONiX SN32F407 & Gowin FPGA)
 *
 * Chức năng:
 *  1. Khởi tạo xung nhịp hệ thống và SysTick 1ms
 *  2. Khởi tạo ngoại vi GPS NEO-6M trên UART0 (9600 baud, P3.1 TX, P3.2 RX)
 *  3. Khởi tạo module 4G LTE SIM A7680C trên UART1 (115200 baud, P2.2 TX, P2.3 RX)
 *  4. Cấu hình GPIO điều khiển LED chẩn đoán (D6, D7, D11) và Còi Buzzer (P3.0, P2.5)
 *  5. Đón nhận tín hiệu kích hoạt Trigger từ:
 *     - Nút bấm SW3 onboard bo MCU Sonix
 *     - Nút FPGA S1/S2 qua Pin 41 -> P2.9 / P2.8 MCU
 *     - Lệnh UART từ máy tính qua P3.11 ('t' / '1' để tự động kích hoạt test)
 *  6. Đón nhận nút bấm Hủy khẩn cấp / Reset trên P2.7 (cọc P27 gần SW6)
 *  7. Thực thi mô hình AI TinyML Autoencoder khi nhận gói tin SPI từ cảm biến
 *  8. Khi kích hoạt tai nạn: Bật còi báo, trích xuất tọa độ GPS và gửi SMS cứu hộ
 *     tới số điện thoại người nhận: 0932591161, phát 3 tiếng bíp báo gửi thành công!
 *****************************************************************************/

#include <SN32F400.h>
#include "system_shared.h"
#include "gps.h"
#include "sim4g.h"
#include "tinyml_infer.h"
#include "debug_uart.h"

#define EMERGENCY_PHONE_NUMBER  "0932591161"

/*=============================  BIẾN TOÀN CỤC  ============================*/
volatile uint32_t g_tick_ms = 0;
volatile int16_t  g_features[4] = {0, 0, 0, 0};
volatile uint8_t  g_spi_pkt_ready = 0;
volatile uint8_t  g_buzzer_enabled = 0;
static volatile uint8_t s_tick_div = 0;

static uint32_t s_last_hb_ms = 0;
static uint32_t s_last_sms_trigger_ms = 0;

/*=============================  SYSTICK & DELAY  ==========================*/
void SysTick_Handler(void)
{
    /* Chia tần 2kHz -> 1kHz (1ms) cho g_tick_ms */
    s_tick_div ^= 1;
    if (s_tick_div) {
        g_tick_ms++;
    }

    /* Tạo xung âm thanh 1000 Hz cho còi thạch anh onboard PZ1 (P3.0) - Dùng BSET/BCLR tránh xung đột Port 3 */
    static uint8_t s_pz1 = 0;
    if (g_buzzer_enabled) {
        s_pz1 ^= 1;
        if (s_pz1) SN_GPIO3->BSET = (1UL << 0);
        else       SN_GPIO3->BCLR = (1UL << 0);
    } else {
        SN_GPIO3->BCLR = (1UL << 0);
    }
}

uint32_t GetTick_ms(void)
{
    return g_tick_ms;
}

void Delay_ms(uint32_t ms)
{
    uint32_t start = g_tick_ms;
    while ((g_tick_ms - start) < ms) {
        GPS_Process();
    }
}

/* Phát tiếng còi báo động với thời lượng ms chỉ định */
void Buzzer_Beep(uint32_t duration_ms)
{
    BUZZER_ON();
    uint32_t start = g_tick_ms;
    while ((g_tick_ms - start) < duration_ms) {
        GPS_Process();
    }
    BUZZER_OFF();
}

/*=============================  CẤU HÌNH GPIO  =============================*/
static void Hardware_GPIO_Init(void)
{
    /* 
     * 1. Cấu hình theo BẢNG 5 file constrain.pdf:
     *    - Còi Buzzer chủ động: Chân P2.5 (Cọc P25 gần SW4) - Output Push-Pull, Active High
     *    - Đèn LED cảnh báo đỏ: Chân P2.6 (Cọc P26 gần SW5) - Output Push-Pull, Active High
     *    - Nút bấm hủy khẩn cấp / Reset: Chân P2.7 (Cọc P27 gần SW6) - Input Pull-up, Active Low
     */
    SN_GPIO2->MODE |= (1UL << 5) | (1UL << 6); // P2.5, P2.6: Output
    SN_GPIO2->BCLR  = (1UL << 5) | (1UL << 6); // Tắt Buzzer P2.5 (0V) và LED P2.6 (0V)

    SN_GPIO2->MODE &= ~(1UL << 7);              // P2.7: Input
    SN_GPIO2->CFG  &= ~(3UL << 14);             // Pull-up cho P2.7 (bits 15:14 = 00b)

    /* 
     * 2. Nhận tín hiệu WAKEUP_IRQ từ Pin 41 FPGA (theo BẢNG 1 file constrain.pdf):
     *    - P2.9: Cụm J7 Chân 5 (P29) -> Input kèm trở kéo xuống (Pull-down)
     *    - P2.8: Cụm J7 Chân 4 (dự phòng) -> Input kèm trở kéo xuống (Pull-down)
     */
    SN_GPIO2->MODE &= ~((1UL << 8) | (1UL << 9));
    SN_GPIO2->CFG = (SN_GPIO2->CFG & ~((3UL << 16) | (3UL << 18))) | ((1UL << 16) | (1UL << 18));

    /* 
     * 3. Nút bấm SW3 onboard bo MCU Sonix EVK (để test gửi tín hiệu tai nạn giả):
     *    - P1.4 (ROW0_P14): Cấu hình Output LOW (0V) để làm mass cho hàng phím 0
     *    - P1.5..P1.7: Cấu hình Input kèm trở kéo lên (Pull-up)
     *    - P2.4 (COL0_P24): Cấu hình Input kèm trở kéo lên (Pull-up: CFG bits 9:8 = 00b)
     */
    SN_GPIO1->MODE |= (1UL << 4);               // P1.4: Output
    SN_GPIO1->BCLR  = (1UL << 4);               // P1.4 = 0V
    SN_GPIO1->MODE &= ~((1UL << 5) | (1UL << 6) | (1UL << 7));
    SN_GPIO1->CFG  &= ~((3UL << 10) | (3UL << 12) | (3UL << 14));

    SN_GPIO2->MODE &= ~(1UL << 4);              // P2.4: Input
    SN_GPIO2->CFG  &= ~(3UL << 8);              // Pull-up chuẩn cho P2.4

    /* P2.10 (INT_P210): Cấu hình Input pull-up */
    SN_GPIO2->MODE &= ~(1UL << 10);
    SN_GPIO2->CFG  &= ~(3UL << 20);

    /* 
     * 4. Đèn LED & Còi onboard Sonix EVK:
     *    - LED D6 (P3.8): Heartbeat (Active Low: 0 = Sáng, 1 = Tắt)
     *    - LED D7 (P3.9): Cảnh báo (Active Low: 0 = Sáng, 1 = Tắt)
     *    - Buzzer PZ1 (P3.0): Còi onboard (Active High: 1 = Kêu, 0 = Tắt)
     */
    SN_GPIO3->MODE |= (1UL << 8) | (1UL << 9) | (1UL << 0);
    SN_GPIO3->BSET  = (1UL << 8) | (1UL << 9); // Tắt D6, D7 mặc định
    SN_GPIO3->BCLR  = (1UL << 0);              // Tắt PZ1 mặc định
}

/*=============================  XỬ LÝ TIN NHẮN SMS  ========================*/
static uint8_t Dispatch_Emergency_SMS(void)
{
    char sms_buffer[160];
    uint32_t i = 0, j = 0;

    const char *prefix = "CANH BAO TAI NAN XE MAY!\n";
    while (prefix[i] != '\0' && j < sizeof(sms_buffer) - 1) {
        sms_buffer[j++] = prefix[i++];
    }

    if (g_gps.fix_valid) {
        const char *t_label = "Toa do: ";
        i = 0;
        while (t_label[i] != '\0' && j < sizeof(sms_buffer) - 1) {
            sms_buffer[j++] = t_label[i++];
        }

        i = 0;
        while (g_gps_coord[i] != '\0' && j < sizeof(sms_buffer) - 1) {
            sms_buffer[j++] = g_gps_coord[i++];
        }

        const char *mid = "\nGoogle Maps: ";
        i = 0;
        while (mid[i] != '\0' && j < sizeof(sms_buffer) - 1) {
            sms_buffer[j++] = mid[i++];
        }

        i = 0;
        while (g_gps_maps_link[i] != '\0' && j < sizeof(sms_buffer) - 1) {
            sms_buffer[j++] = g_gps_maps_link[i++];
        }
    } else {
        const char *no_fix = "GPS: CHUA KHOA VE TINH (Fix=0)\n(Vui long de anten ra gan cua so hoac ngoai troi)";
        i = 0;
        while (no_fix[i] != '\0' && j < sizeof(sms_buffer) - 1) {
            sms_buffer[j++] = no_fix[i++];
        }
    }

    sms_buffer[j] = '\0';

    DEBUG_UART_SendString("[SMS] START: Preparing to dispatch emergency SMS...\r\n");
    if (g_gps.fix_valid) {
        DEBUG_UART_Printf("[SMS] GPS Coordinates: %s\r\n", g_gps_coord);
    } else {
        DEBUG_UART_SendString("[SMS] GPS: No Satellite Fix (Fix=0)\r\n");
    }

    /* Bật LED D11 báo trạng thái gửi 4G */
    LED_D11_4G_ON();

    /* Gửi tin nhắn SMS qua SIM A7680C */
    uint8_t ok = SIM4G_SendSMS(EMERGENCY_PHONE_NUMBER, sms_buffer);

    LED_D11_4G_OFF();

    return ok;
}

/* Thực thi chu trình kích hoạt khẩn cấp khi có sự kiện (nút nhấn / FPGA / UART) */
static void Execute_Emergency_Routine(const char *source_desc)
{
    DEBUG_UART_Printf("[EMERGENCY] Triggered by: %s\r\n", source_desc);

    /* Bíp 100ms báo đã nhận lệnh */
    Buzzer_Beep(100);

    /* Bật đèn cảnh báo trong quá trình gửi */
    ALERT_LED_ON();

    /* Gửi tin nhắn SMS cứu hộ khẩn cấp tới 0932591161 */
    uint8_t sms_ok = Dispatch_Emergency_SMS();

    ALERT_LED_OFF();

    if (sms_ok) {
        DEBUG_UART_SendString("[SMS] Result: SUCCESS (3 beeps emitted)\r\n");
        /* Bíp 3 tiếng ngắn (bíp-bíp-bíp) báo gửi tin nhắn SMS thành công! */
        Buzzer_Beep(120);
        Delay_ms(80);
        Buzzer_Beep(120);
        Delay_ms(80);
        Buzzer_Beep(120);
    } else {
        DEBUG_UART_SendString("[SMS] Result: FAILED (Long error beep emitted)\r\n");
        /* Bíp 1 tiếng dài 600ms cảnh báo lỗi SIM hoặc chưa bắt sóng */
        Buzzer_Beep(600);
    }
}

/*=============================  HÀM MAIN  =================================*/
int main(void)
{
    /* Vô hiệu hóa Watchdog và LVD reset trong quá trình khởi động */
    WDT_CFG = 0x5AFA0000UL;
    SYS0_LVDCTRL = 0;

    /* 1. Khởi tạo UART Debug trên P3.10 TX, P3.11 RX (115200 8N1) */
    DEBUG_UART_Init();
    DEBUG_UART_SendString("\r\n========================================\r\n");
    DEBUG_UART_SendString("   HE THONG PHAT HIEN TAI NAN XE MAY\r\n");
    DEBUG_UART_SendString("   MCU: SONiX SN32F407 | FPGA: GW1NSR-4C\r\n");
    DEBUG_UART_SendString("   So dien thoai cuu ho: 0932591161\r\n");
    DEBUG_UART_SendString("========================================\r\n");
    DEBUG_UART_SendString("[BOOT] SONiX SN32F407 Booting...\r\n");
    DEBUG_UART_SendString("[DEBUG_UART] P3.10 TX / P3.11 RX (115200 8N1) OK\r\n");

    /* 2. Khởi tạo SysTick ngắt 2kHz (IHRC 12MHz) để tạo xung âm thanh 1kHz cho còi PZ1 */
    SysTick_Config(12000000UL / 2000UL);

    /* 3. Cấu hình GPIO chẩn đoán và còi cảnh báo */
    Hardware_GPIO_Init();
    DEBUG_UART_SendString("[GPIO] OK\r\n");

    /* Đảm bảo tuyệt đối còi TẮT lúc cắm nguồn */
    BUZZER_OFF();

    /* Phát 1 tiếng bíp ngắn 80ms và nháy LED để báo hiệu MCU khởi động / Reset thành công */
    LED_D6_ON();
    ALERT_LED_ON();
    Buzzer_Beep(80);
    LED_D6_OFF();
    ALERT_LED_OFF();
    BUZZER_OFF();

    /* 4. Khởi tạo mô hình TinyML */
    tinyml_init();

    /* 5. Khởi tạo ngoại vi GPS NEO-6M trên UART0 */
    GPS_Init();
    DEBUG_UART_SendString("[GPS] UART0 9600 OK\r\n");

    /* 6. Khởi tạo Module 4G LTE SIM A7680C trên UART1 */
    DEBUG_UART_SendString("[MODEM] Initializing SIM A7680C (UART1 115200)...\r\n");
    SIM4G_Init();
    if (g_sim_ready) {
        DEBUG_UART_SendString("[MODEM] Ready (CPIN OK)\r\n");
    } else {
        DEBUG_UART_SendString("[MODEM] WARN: Not Ready or SIM Missing\r\n");
    }

    /* Đảm bảo còi và đèn cảnh báo đã TẮT trước khi vào vòng lặp chính */
    BUZZER_OFF();
    ALERT_LED_OFF();

    s_last_hb_ms = g_tick_ms;
    DEBUG_UART_SendString("[SYS] Ready. Listening for Button / Trigger events.\r\n");
    DEBUG_UART_SendString("========================================\r\n\r\n");

    while (1)
    {
        uint32_t now = g_tick_ms;

        /* A. Duy trì cỗ máy trạng thái tách chuỗi NMEA từ GPS */
        GPS_Process();

        /* B. Nhịp tim hệ thống Heartbeat trên LED D6 (P3.8):
         * - Khi chưa có GPS Fix: Chớp tắt 500ms
         * - Khi đã khóa vệ tinh GPS (Fix valid): Chớp tắt nhanh 200ms
         */
        uint32_t hb_period = g_gps.fix_valid ? 200 : 500;
        if ((now - s_last_hb_ms) >= hb_period) {
            s_last_hb_ms = now;
            /* Đảo trạng thái LED D6 (P3.8) */
            if (SN_GPIO3->DATA & (1UL << 8)) {
                LED_D6_ON();
            } else {
                LED_D6_OFF();
            }
        }

        /* Gửi nhịp tim UART định kỳ mỗi 2000ms để quan sát hệ thống trên COM */
        {
            static uint32_t s_last_uart_hb = 0;
            if ((now - s_last_uart_hb) >= 2000) {
                s_last_uart_hb = now;
                DEBUG_UART_Printf("[SYS] Heartbeat | Tick: %u ms | GPS Fix: %d (Sats: %u) | SIM Ready: %d\r\n",
                                  now, g_gps.fix_valid, g_gps.satellites, g_sim_ready);
            }
        }

        /* C. BẮT LỆNH ĐIỀU KHIỂN TỪ MÁY TÍNH QUA DEBUG UART (P3.11 RX) */
        if (DEBUG_UART_Available()) {
            char cmd = DEBUG_UART_ReadChar();
            if (cmd == 't' || cmd == 'T' || cmd == '1') {
                Execute_Emergency_Routine("PC Serial Command ('t')");
            } else if (cmd == 's' || cmd == 'S') {
                DEBUG_UART_Printf("[STATUS] Tick=%u ms, GPS Fix=%d, Sats=%u, Lat=%s, Lon=%s, SIM=%d\r\n",
                                  now, g_gps.fix_valid, g_gps.satellites,
                                  g_gps.latitude, g_gps.longitude, g_sim_ready);
            }
        }

        /* D. BẮT SỰ KIỆN NÚT BẤM TEST SW3 (P2.4) TRÊN BO MẠCH SONIX EVK:
         * Mỗi lần bấm nút SW3: Còi BÍP ngay lập tức và gửi SMS khẩn cấp (ổn định 100% mọi lần bấm)!
         */
        if ((SN_GPIO2->DATA & (1UL << 4)) == 0) {
            Delay_ms(25); /* Chống dội phím */
            if ((SN_GPIO2->DATA & (1UL << 4)) == 0) {
                DEBUG_UART_SendString("\r\n>>> [BUTTON] SW3 PRESSED (Test Emergency Trigger) <<<\r\n");

                /* Kích hoạt ngay chu trình khẩn cấp: Bíp còi + Bật đèn + Gửi SMS */
                Execute_Emergency_Routine("SW3 Onboard Button");

                /* Đợi người dùng thả phím SW3 ra kèm TIMEOUT an toàn để sẵn sàng cho lần bấm tiếp theo */
                uint32_t hold_start = GetTick_ms();
                while (((SN_GPIO2->DATA & (1UL << 4)) == 0) && ((GetTick_ms() - hold_start) < 1500)) {
                    GPS_Process();
                }
                Delay_ms(100); /* Chống dội khi nhả phím */
                DEBUG_UART_SendString("[BUTTON] SW3 Ready for next press.\r\n");
            }
        }

        /* E. BẮT SỰ KIỆN XUNG TỪ FPGA (Pin 41 -> P2.9 hoặc P2.8) KHI BẤM NÚT S1/S2 TRÊN BO FPGA: */
        if ((SN_GPIO2->DATA & (1UL << 9)) || (SN_GPIO2->DATA & (1UL << 8))) {
            Delay_ms(25);
            if ((SN_GPIO2->DATA & (1UL << 9)) || (SN_GPIO2->DATA & (1UL << 8))) {
                DEBUG_UART_SendString("\r\n>>> [FPGA] Signal detected from FPGA (Pin 41 -> P2.9/P2.8) <<<\r\n");
                Execute_Emergency_Routine("FPGA S1/S2 Trigger (Pin 41)");

                uint32_t pulse_start = GetTick_ms();
                while (((SN_GPIO2->DATA & (1UL << 9)) || (SN_GPIO2->DATA & (1UL << 8))) && ((GetTick_ms() - pulse_start) < 1500)) {
                    GPS_Process();
                }
                Delay_ms(100);
            }
        }

        /* F. NÚT BẤM HỦY / RESET KHẨN CẤP (P2.7): Tắt để tránh xung đột ma trận phím onboard */
        /* (Để hệ thống luôn tập trung phục vụ nút báo động khẩn cấp SW3 và xung FPGA) */

        /* G. KIỂM TRA GÓI TIN SPI TỪ FPGA NẾU CÓ DỮ LIỆU ĐẶC TRƯNG THỰC TẾ */
        if (g_spi_pkt_ready) {
            g_spi_pkt_ready = 0;
            tinyml_features_t feat;
            feat.pitch    = (float)g_features[0] / 256.0f;
            feat.roll     = (float)g_features[1] / 256.0f;
            feat.rms      = (float)g_features[2] / 256.0f;
            feat.max_jerk = (float)g_features[3] / 16.0f;

            float mse_score = tinyml_predict(&feat);
            if (tinyml_confirm_accident(feat.roll, mse_score, 50)) {
                if ((now - s_last_sms_trigger_ms) >= 3000 || s_last_sms_trigger_ms == 0) {
                    s_last_sms_trigger_ms = now;
                    Execute_Emergency_Routine("TinyML Accident Detection");
                }
            }
        }
    }
}
