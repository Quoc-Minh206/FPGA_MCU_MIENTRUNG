/******************************************************************************
 * main.c  -  Hệ thống phát hiện tai nạn xe máy TinyML (SONiX SN32F407 & Gowin FPGA)
 *
 * Chức năng:
 *  1. Khởi tạo xung nhịp hệ thống và SysTick 1ms
 *  2. Khởi tạo ngoại vi GPS NEO-6M trên UART0 (9600 baud, P3.1 TX, P3.2 RX)
 *  3. Khởi tạo module 4G LTE SIM A7680C trên UART1 (115200 baud, P2.2 TX, P2.3 RX, Reset P2.1)
 *  4. Cấu hình GPIO điều khiển LED chẩn đoán (D8, D9, D10, D11) và Còi Buzzer (P3.0, P2.5)
 *  5. Đón nhận tín hiệu kích hoạt Trigger từ nút bấm SW3 onboard hoặc nút FPGA S1/S2 qua P2.9
 *  6. Thực thi mô hình AI TinyML Autoencoder khi nhận gói tin SPI
 *  7. Khi kích hoạt tai nạn: Bật còi báo động nhịp điệu, trích xuất tọa độ GPS và gửi SMS cứu hộ
 *     tới số điện thoại người nhận: 0932591161
 *****************************************************************************/

#include <SN32F400.h>
#include "system_shared.h"
#include "gps.h"
#include "sim4g.h"
#include "tinyml_infer.h"

#define EMERGENCY_PHONE_NUMBER  "0932591161"

/*=============================  BIẾN TOÀN CỤC  ============================*/
volatile uint32_t g_tick_ms = 0;
volatile int16_t  g_features[4] = {0, 0, 0, 0};
volatile uint8_t  g_spi_pkt_ready = 0;
volatile uint8_t  g_buzzer_enabled = 0;
static volatile uint8_t s_tick_div = 0;

/* Trạng thái hệ thống */
typedef enum {
    SYS_STATE_NORMAL = 0,
    SYS_STATE_ACCIDENT_CONFIRMED,
    SYS_STATE_ALERT_DISPATCHED
} system_state_t;

static system_state_t s_system_state = SYS_STATE_NORMAL;
static uint32_t s_last_hb_ms = 0;
static uint32_t s_last_sms_try_ms = 0;

/*=============================  SYSTICK & DELAY  ==========================*/
void SysTick_Handler(void)
{
    /* Chia tần 2kHz -> 1kHz (1ms) cho g_tick_ms */
    s_tick_div ^= 1;
    if (s_tick_div) {
        g_tick_ms++;
    }

    /* Tạo xung âm thanh 1000 Hz cho còi thạch anh onboard PZ1 (P3.0) */
    if (g_buzzer_enabled) {
        SN_GPIO3->DATA ^= (1UL << 0);
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
        /* Duy trì xử lý GPS trong lúc delay */
        GPS_Process();

        /* Nếu đang ở trạng thái báo động tai nạn, duy trì bíp ngắt quãng văn minh:
         * - Khi chưa gửi SMS: Bíp dồn dập (150ms kêu / 250ms ngắt)
         * - Khi đã gửi SMS thành công: Bíp chậm giãn cách (200ms kêu / 800ms ngắt)
         */
        if (s_system_state != SYS_STATE_NORMAL) {
            if (!g_sim_sms_sent) {
                if ((g_tick_ms % 400) < 150) {
                    BUZZER_ON();
                } else {
                    BUZZER_OFF();
                }
            } else {
                if ((g_tick_ms % 1000) < 200) {
                    BUZZER_ON();
                } else {
                    BUZZER_OFF();
                }
            }
        }
    }
}

/*=============================  CẤU HÌNH GPIO  =============================*/
static void Hardware_GPIO_Init(void)
{
    /* 
     * 1. Cấu hình theo BẢNG 5 file constrain.pdf:
     *    - Còi Buzzer chủ động: Chân P2.5 (Cọc P25 gần SW4) - Output Push-Pull, Active High
     *    - Đèn LED cảnh báo đỏ: Chân P2.6 (Cọc P26 gần SW5) - Output Push-Pull, Active High
     *    - Nút bấm khẩn cấp/hủy: Chân P2.7 (Cọc P27 gần SW6) - Input kèm trở kéo lên (Pull-up), Active Low
     */
    SN_GPIO2->MODE |= (1UL << 5) | (1UL << 6); // P2.5, P2.6: Output
    SN_GPIO2->BCLR  = (1UL << 5) | (1UL << 6); // Mặc định tắt Buzzer P2.5 (0V) và LED P2.6 (0V)

    SN_GPIO2->MODE &= ~(1UL << 7);              // P2.7: Input
    SN_GPIO2->CFG  &= ~(3UL << 14);             // CFG bits 15:14 = 00b (Pull-up chuẩn cho P2.7)

    /* 
     * 2. Nhận tín hiệu WAKEUP_IRQ từ Pin 41 FPGA (theo BẢNG 1 file constrain.pdf):
     *    - P2.9: Cụm J7 Chân 5 (P29) -> Input kèm trở kéo xuống (Pull-down)
     *    - P2.8: Cụm J7 Chân 4 (dự phòng) -> Input kèm trở kéo xuống (Pull-down)
     *    Khi FPGA không kích hoạt hoặc chưa cắm cáp, chân P2.9 luôn được kéo về 0V, chống nhiễu tuyệt đối.
     */
    SN_GPIO2->MODE &= ~((1UL << 8) | (1UL << 9));
    SN_GPIO2->CFG = (SN_GPIO2->CFG & ~((3UL << 16) | (3UL << 18))) | ((1UL << 16) | (1UL << 18)); // Pull-down cho P2.8 và P2.9

    /* 
     * 3. Nút bấm SW3 onboard bo MCU Sonix EVK (để test gửi tín hiệu tai nạn giả):
     *    - P1.4 (ROW0_P14): Cấu hình Output LOW (0V) để làm mass cho hàng phím 0
     *    - P1.5..P1.7: Cấu hình Input kèm trở kéo lên (Pull-up)
     *    - P2.4 (COL0_P24): Cấu hình Input kèm trở kéo lên (Pull-up: CFG bits 9:8 = 00b)
     *    Khi bấm SW3: P2.4 nối với P1.4 (0V) -> DATA bit 4 = 0 (kích hoạt báo động test)
     *    Khi nhả SW3: P2.4 kéo về 3.3V -> DATA bit 4 = 1
     */
    SN_GPIO1->MODE |= (1UL << 4);               // P1.4: Output
    SN_GPIO1->BCLR  = (1UL << 4);               // P1.4 = 0V
    SN_GPIO1->MODE &= ~((1UL << 5) | (1UL << 6) | (1UL << 7)); // P1.5..P1.7: Input
    SN_GPIO1->CFG  &= ~((3UL << 10) | (3UL << 12) | (3UL << 14)); // Pull-up cho P1.5..P1.7

    SN_GPIO2->MODE &= ~(1UL << 4);              // P2.4: Input
    SN_GPIO2->CFG  &= ~(3UL << 8);              // CFG bits 9:8 = 00b (Pull-up chuẩn cho P2.4)

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
static void Dispatch_Emergency_SMS(void)
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

        const char *mid = "\nMaps: ";
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

    /* Bật LED D11 báo trạng thái gửi 4G */
    LED_D11_4G_ON();

    /* Gửi tin nhắn SMS qua SIM A7680C */
    SIM4G_SendSMS(EMERGENCY_PHONE_NUMBER, sms_buffer);

    LED_D11_4G_OFF();
}

/*=============================  HÀM MAIN  =================================*/
int main(void)
{
    /* Vô hiệu hóa Watchdog và LVD reset trong quá trình khởi động để chống treo/reset bất ngờ */
    WDT_CFG = 0x5AFA0000UL;
    SYS0_LVDCTRL = 0;

    /* 1. Khởi tạo SysTick ngắt 2kHz (IHRC 12MHz) để tạo xung âm thanh 1kHz cho còi PZ1 */
    SysTick_Config(12000000UL / 2000UL);

    /* 2. Cấu hình GPIO chẩn đoán và còi cảnh báo */
    Hardware_GPIO_Init();

    /* Đảm bảo tuyệt đối còi TẮT lúc cắm nguồn (KHÔNG KÊU CÒI KHI VỪA CẮM ĐIỆN) */
    BUZZER_OFF();

    /* Nháy đèn D6 & D7 trong 200ms để xác nhận MCU đã nạp và khởi động thành công */
    LED_D6_ON();
    ALERT_LED_ON();
    Delay_ms(200);
    LED_D6_OFF();
    ALERT_LED_OFF();
    BUZZER_OFF();

    /* 3. Khởi tạo mô hình TinyML */
    tinyml_init();

    /* 4. Khởi tạo ngoại vi GPS NEO-6M trên UART0 */
    GPS_Init();

    /* 5. Khởi tạo Module 4G LTE SIM A7680C trên UART1 */
    SIM4G_Init();

    /* Chờ 500ms để toàn bộ các chân tín hiệu ổn định hoàn toàn */
    Delay_ms(500);

    /* Đảm bảo còi và đèn cảnh báo đã TẮT trước khi vào vòng lặp chính */
    BUZZER_OFF();
    ALERT_LED_OFF();
    s_system_state = SYS_STATE_NORMAL;

    s_last_hb_ms = g_tick_ms;
    s_last_sms_try_ms = 0;

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

        /* C. Kiểm tra nút HỦY báo động khẩn cấp (P2.7 cọc P27 gần SW6, Active Low) */
        if ((SN_GPIO2->DATA & (1UL << 7)) == 0) {
            s_system_state = SYS_STATE_NORMAL;
            g_sim_sms_sent = 0;
            ALERT_LED_OFF();
            BUZZER_OFF();
            Delay_ms(300); // Chống dội phím hủy
        }

        /* D. Bắt sự kiện kích hoạt tai nạn mới:
         * 1. Nút SW3 trên bo MCU (Chân P2.4, Active Low khi nhấn)
         * 2. Xung WAKEUP_IRQ từ FPGA (Chân P2.9 cọc J7 Pin 5, Active High)
         * 3. Gói tin SPI từ FPGA (g_spi_pkt_ready)
         */
        uint8_t trigger_event = 0;

        /* 1. Kiểm tra nút bấm SW3 trên bo MCU */
        if ((SN_GPIO2->DATA & (1UL << 4)) == 0) {
            Delay_ms(30); /* Chống dội phím 30ms */
            if ((SN_GPIO2->DATA & (1UL << 4)) == 0) {
                trigger_event = 1;
                /* Chờ người dùng thả nút SW3 ra để mỗi lần nhấn là 1 lần gửi mới */
                while ((SN_GPIO2->DATA & (1UL << 4)) == 0) {
                    GPS_Process();
                }
            }
        }

        /* 2. Kiểm tra tín hiệu từ FPGA (Pin 41 -> P2.9) khi bấm S1/S2 trên FPGA */
        if (SN_GPIO2->DATA & (1UL << 9)) {
            trigger_event = 1;
        }

        /* 3. Kiểm tra gói tin SPI từ FPGA nếu có */
        if (g_spi_pkt_ready) {
            g_spi_pkt_ready = 0;
            tinyml_features_t feat;
            feat.pitch    = (float)g_features[0] / 256.0f;
            feat.roll     = (float)g_features[1] / 256.0f;
            feat.rms      = (float)g_features[2] / 256.0f;
            feat.max_jerk = (float)g_features[3] / 16.0f;

            float mse_score = tinyml_predict(&feat);
            if (tinyml_confirm_accident(feat.roll, mse_score, 50)) {
                trigger_event = 1;
            }
        }

        /* Xử lý khi có sự kiện kích hoạt tai nạn */
        if (trigger_event) {
            ALERT_LED_ON(); // Bật LED cảnh báo (P2.6 + D7)
            BUZZER_ON();    // Bật còi báo động ngay lập tức
            s_system_state    = SYS_STATE_ACCIDENT_CONFIRMED;
            g_sim_sms_sent    = 0; // Đánh dấu chưa gửi để cho phép gửi SMS
            s_last_sms_try_ms = 0; // Kích hoạt gửi SMS ngay lập tức!
        }

        /* E. Xử lý trạng thái Tai nạn được xác nhận */
        if (s_system_state == SYS_STATE_ACCIDENT_CONFIRMED || s_system_state == SYS_STATE_ALERT_DISPATCHED) {
            s_system_state = SYS_STATE_ALERT_DISPATCHED;

            /* Bật LED cảnh báo tai nạn */
            ALERT_LED_ON();

            /* Còi báo động ngắt quãng văn minh:
             * - Khi chưa gửi SMS: Bíp dồn dập (150ms kêu / 250ms ngắt)
             * - Khi đã gửi SMS thành công: Bíp chậm giãn cách (200ms kêu / 800ms ngắt)
             */
            if (!g_sim_sms_sent) {
                if ((now % 400) < 150) {
                    BUZZER_ON();
                } else {
                    BUZZER_OFF();
                }
            } else {
                if ((now % 1000) < 200) {
                    BUZZER_ON();
                } else {
                    BUZZER_OFF();
                }
            }

            /* Gửi SMS cứu hộ ngay lập tức khi kích hoạt (s_last_sms_try_ms = 0),
             * hoặc tự động gửi lại sau mỗi 10 giây nếu lần gửi trước chưa thành công */
            if (!g_sim_sms_sent && (s_last_sms_try_ms == 0 || (now - s_last_sms_try_ms >= 10000))) {
                s_last_sms_try_ms = now;
                Dispatch_Emergency_SMS();

                if (g_sim_sms_sent) {
                    /* Gửi tin nhắn thành công: Tắt còi báo động, tắt đèn cảnh báo */
                    BUZZER_OFF();
                    ALERT_LED_OFF();

                    /* Bíp 3 tiếng ngắn (bíp - bíp - bíp) báo gửi thành công */
                    Delay_ms(200);
                    BUZZER_ON(); Delay_ms(120); BUZZER_OFF(); Delay_ms(100);
                    BUZZER_ON(); Delay_ms(120); BUZZER_OFF(); Delay_ms(100);
                    BUZZER_ON(); Delay_ms(120); BUZZER_OFF(); Delay_ms(200);

                    /* Tự động Reset về trạng thái bình thường để CHO PHÉP BẤM GỬI LẠI TIẾP TỤC! */
                    s_system_state    = SYS_STATE_NORMAL;
                    g_sim_sms_sent    = 0;
                    s_last_sms_try_ms = 0;
                }
            }
        } else {
            /* Trạng thái bình thường: Tuyệt đối TẮT còi và TẮT đèn cảnh báo */
            BUZZER_OFF();
            ALERT_LED_OFF();
        }
    }
}
