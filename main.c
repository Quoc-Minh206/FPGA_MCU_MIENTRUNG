/******************************************************************************
 * main.c  -  Hệ thống phát hiện tai nạn xe máy TinyML (SONiX SN32F407 & Gowin FPGA)
 *
 * Kiến trúc: Bare-metal C thuần (Không RTOS, Không <string.h>, Không <stdio.h>)
 * Vi điều khiển: SONiX SN32F407 (ARM Cortex-M0, 32KB Flash ROM, 8KB SRAM)
 * Tiền xử lý: Gowin FPGA OneKiwi 4K (GW1NSR-LV4C) + IMU MPU6050
 * Định vị & Viễn thông: GPS u-blox NEO-M6N (UART0) + 4G SIM A7680C (UART1)
 *****************************************************************************/

#include <SN32F400.h>
#include "system_shared.h"
#include "gps.h"
#include "sim4g.h"
#include "tinyml_infer.h"
#include "debug_uart.h"

#define EMERGENCY_PHONE_NUMBER  "+84792394699"

/*=============================  BIẾN TOÀN CỤC  ============================*/
volatile uint32_t g_tick_ms = 0;
volatile int16_t  g_features[4] = {0, 0, 0, 0}; /* Pitch, Roll, RMS, Variance */
volatile uint8_t  g_spi_pkt_ready = 0;
volatile uint8_t  g_buzzer_enabled = 0;
volatile uint8_t  g_periph_init_done = 0;
static volatile uint8_t s_tick_div = 0;

static uint32_t s_last_hb_ms = 0;
static uint32_t s_last_sms_trigger_ms = 0;

/*=============================  SYSTICK & DELAY  ==========================*/
/*
 * Ngắt SysTick chạy ở tần số 2000 Hz (0.5 ms):
 * - Tạo xung vuông 1000 Hz (chu kỳ 1 ms) kích còi thụ động PZ1 tại chân P3.0.
 * - Sử dụng thanh ghi nguyên tử BSET / BCLR để tránh xung đột với các chân khác trên Port 3.
 * - Chia đôi tần số bằng biến s_tick_div để tăng bộ đếm thời gian g_tick_ms mỗi 1 ms.
 */
void SysTick_Handler(void)
{
    s_tick_div ^= 1U;
    if (s_tick_div) {
        g_tick_ms++;
    }

    static uint8_t s_pz1 = 0;
    if (g_buzzer_enabled) {
        s_pz1 ^= 1U;
        if (s_pz1) {
            SN_GPIO3->BSET = (1UL << 0);
        } else {
            SN_GPIO3->BCLR = (1UL << 0);
        }
    } else {
        SN_GPIO3->BCLR = (1UL << 0);
    }
}

uint32_t GetTick_ms(void)
{
    return g_tick_ms;
}

/*
 * Hàm trễ không chặn (Non-blocking đối với GPS):
 * Trong suốt thời gian chờ, MCU liên tục rút dữ liệu từ UART0 để không làm rơi
 * bất kỳ ký tự NMEA nào từ module GPS NEO-M6N.
 */
void Delay_ms(uint32_t ms)
{
    uint32_t start = g_tick_ms;
    while ((g_tick_ms - start) < ms) {
        if (g_periph_init_done) {
            GPS_Process();
            SIM4G_Poll();
        }
    }
}

/* Phát tiếng còi báo động với thời lượng ms chỉ định */
void Buzzer_Beep(uint32_t duration_ms)
{
    BUZZER_ON();
    uint32_t start = g_tick_ms;
    while ((g_tick_ms - start) < duration_ms) {
        if (g_periph_init_done) {
            GPS_Process();
            SIM4G_Poll();
        }
    }
    BUZZER_OFF();
}

/*=============================  CẤU HÌNH GPIO  =============================*/
/*
 * Hàm khôi phục đường MASS (0V) cho hàng phím 0 (P1.4) và Pull-up cho cột 0 (P2.4).
 * Lý do vật lý: Trên bo mạch 32F407_EVK_V1.0, nút SW3 nối giữa P2.4 (COL0) và
 * P1.4 (ROW0). Khóa cứng lại thanh ghi sau mỗi chu trình để nút luôn luôn nhạy.
 */
void Restore_SW3_Matrix_GPIO(void)
{
    /* P1.4 (ROW0): Output Push-Pull, ép cứng xuống 0V bằng thanh ghi nguyên tử BCLR */
    SN_GPIO1->MODE |= (1UL << 4);
    SN_GPIO1->BCLR  = (1UL << 4);

    /* P2.4 (COL0): Input (MODE bit 4 = 0), bật điện trở kéo lên (CFG bits 9:8 = 00b) */
    SN_GPIO2->MODE &= ~(1UL << 4);
    SN_GPIO2->CFG  &= ~(3UL << 8);

    /* P2.8, P2.9 (Nhận xung WAKEUP_IRQ từ Pin 41 FPGA): Input Pull-down (CFG = 01b) */
    SN_GPIO2->MODE &= ~((1UL << 8) | (1UL << 9));
    SN_GPIO2->CFG   = (SN_GPIO2->CFG & ~((3UL << 16) | (3UL << 18)))
                    | ((1UL << 16) | (1UL << 18));
}

static void Hardware_GPIO_Init(void)
{
    /*
     * 1. Cấu hình các chân cột phím P2.5 (COL1) và P2.6 (COL2) làm Input Pull-up:
     *    Tuyệt đối không để Output mức High để tránh ngắn mạch với chân quét hàng P1.4 (0V).
     */
    SN_GPIO2->MODE &= ~((1UL << 5) | (1UL << 6));
    SN_GPIO2->CFG  &= ~((3UL << 10) | (3UL << 12));

    /*
     * 2. Cấu hình các chân ma trận phím P1.5..P1.7 và P2.10:
     *    - GPIOn_CFG (Offset 0x08): Mỗi chân chiếm 2 bit:
     *      + 00b: Bật điện trở kéo lên (Pull-up)
     *      + 01b: Bật điện trở kéo xuống (Pull-down)
     *      + 10b: Floating (Mặc định lúc Reset là 0x00AAAAAA)
     */
    SN_GPIO1->MODE &= ~((1UL << 5) | (1UL << 6) | (1UL << 7));
    SN_GPIO1->CFG  &= ~((3UL << 10) | (3UL << 12) | (3UL << 14));

    SN_GPIO2->MODE &= ~(1UL << 10);
    SN_GPIO2->CFG  &= ~(3UL << 20);

    /* Khởi tạo tổ hợp phím SW3 (P1.4 + P2.4) và chân IRQ FPGA (P2.8, P2.9) */
    Restore_SW3_Matrix_GPIO();

    /*
     * 3. Đèn LED & Còi onboard Sonix EVK (Port 3):
     *    - LED D6 (P3.8) & LED D7 (P3.9): Active Low (Ghi BSET = 1 để tắt, BCLR = 1 để sáng)
     *    - Buzzer PZ1 (P3.0): Active High (Ghi BCLR = 1 để tắt)
     */
    SN_GPIO3->MODE |= (1UL << 8) | (1UL << 9) | (1UL << 0);
    SN_GPIO3->BSET  = (1UL << 8) | (1UL << 9);
    SN_GPIO3->BCLR  = (1UL << 0);
}

/*=============================  XỬ LÝ TIN NHẮN SMS  ========================*/
/* Hàm nối chuỗi tối giản không dùng <string.h> để tiết kiệm ROM */
static uint32_t Append_Str(char *dst, uint32_t max_len, uint32_t idx, const char *src)
{
    uint32_t i = 0;
    while ((src[i] != '\0') && (idx < (max_len - 1U))) {
        dst[idx++] = src[i++];
    }
    dst[idx] = '\0';
    return idx;
}

static uint8_t Dispatch_Emergency_SMS(void)
{
    char sms_buffer[160];
    uint32_t len = 0;

        /*
     * LUU Y VE BO LOC TIN NHAN TAI VIET NAM (Nghi dinh 91/2020/ND-CP):
     * Cac nha mang (Viettel, MobiFone, VinaPhone) chan toan bo tin nhan ca nhan co link URL.
     * Do do, chi gui van ban thuan va toa do chuan, khong kem link URL https://
     */
    len = Append_Str(sms_buffer, sizeof(sms_buffer), len, "SOS TAI NAN XE MAY! ");

    if (g_gps.fix_valid) {
        len = Append_Str(sms_buffer, sizeof(sms_buffer), len, "Toa do: ");
        len = Append_Str(sms_buffer, sizeof(sms_buffer), len, g_gps_coord);
    } else {
        if (g_gps_coord[0] != '\0' && g_gps_coord[0] != 'N') {
            len = Append_Str(sms_buffer, sizeof(sms_buffer), len, "Toa do: ");
            len = Append_Str(sms_buffer, sizeof(sms_buffer), len, g_gps_coord);
        } else {
            len = Append_Str(sms_buffer, sizeof(sms_buffer), len, "GPS: Chua khoa ve tinh");
        }
    }
    len = Append_Str(sms_buffer, sizeof(sms_buffer), len, ". Xin ho tro khan cap!");

    DEBUG_UART_SendString("[SMS] START: Preparing to dispatch emergency SMS...\r\n");
    if (g_gps.fix_valid) {
        DEBUG_UART_Printf("[SMS] GPS Coordinates: %s\r\n", g_gps_coord);
    } else {
        DEBUG_UART_SendString("[SMS] GPS: No Satellite Fix (Fix=0)\r\n");
    }

    /* Bật LED D11 báo trạng thái đang truyền 4G */
    LED_D11_4G_ON();

    /* Gửi tin nhắn SMS lần 1 tới số quốc tế +84 */
    uint8_t ok = SIM4G_SendSMS("+84792394699", sms_buffer);
    Delay_ms(1000);
    /* Gửi thêm lần 2 tới số nội địa 0792394699 */
    SIM4G_SendSMS("0792394699", sms_buffer);

    LED_D11_4G_OFF();

    /* Khôi phục cứng chân P1.4 = 0V và P2.4 Pull-up ngay sau khi kết thúc gửi */
    Restore_SW3_Matrix_GPIO();

    return ok;
}

/* Thực thi chu trình kích hoạt khẩn cấp khi có sự kiện (nút nhấn / FPGA / UART) */
static void Execute_Emergency_Routine(const char *source_desc)
{
    DEBUG_UART_Printf("[EMERGENCY] Triggered by: %s\r\n", source_desc);

    /* Bíp 100ms báo đã nhận lệnh kích hoạt */
    Buzzer_Beep(100);

    /* Bật đèn cảnh báo đỏ P2.6 trong quá trình gửi */
    ALERT_LED_ON();

    /* Gửi tin nhắn SMS cứu hộ khẩn cấp tới 0932591161 */
    uint8_t sms_ok = Dispatch_Emergency_SMS();

    ALERT_LED_OFF();

    /* Đảm bảo cấu hình GPIO của ma trận nút nhấn SW3 không bị lệch sau khi tắt đèn/còi */
    Restore_SW3_Matrix_GPIO();

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
    /*
     * Vô hiệu hóa Watchdog Timer (WDT_CFG Base: 0x40010000) và LVD Reset:
     * Phải ghi kèm khóa bảo vệ WDKEY = 0x5AFA vào bits [31:16] và WDTEN = 0.
     */
    WDT_CFG = 0x5AFA0000UL;
    SYS0_LVDCTRL = 0UL;

    /* Cấp xung nhịp AHB cho UART0 và UART1 trước để bảo vệ chống HardFault */
    SN_SYS1->AHBCLKEN |= (1UL << 16) | (1UL << 17);

    /* 1. Khởi tạo UART Debug trên P3.10 TX, P3.11 RX (115200 8N1) */
    DEBUG_UART_Init();
    DEBUG_UART_SendString("\r\n========================================\r\n");
    DEBUG_UART_SendString("   HE THONG PHAT HIEN TAI NAN XE MAY\r\n");
    DEBUG_UART_SendString("   MCU: SONiX SN32F407 | FPGA: GW1NSR-4C\r\n");
    DEBUG_UART_SendString("   So dien thoai cuu ho: +84792394699\r\n");
    DEBUG_UART_SendString("========================================\r\n");

    /* 2. Khởi tạo SysTick ngắt 2kHz (IHRC 12MHz / 2000 = 6000 chu kỳ) */
    SysTick_Config(12000000UL / 2000UL);

    /* 3. Cấu hình GPIO chẩn đoán, nút nhấn và còi cảnh báo */
    Hardware_GPIO_Init();
    BUZZER_OFF();

    /* Phát 1 tiếng bíp ngắn 80ms và nháy LED báo hiệu MCU khởi động thành công */
    LED_D6_ON();
    ALERT_LED_ON();
    Buzzer_Beep(80);
    LED_D6_OFF();
    ALERT_LED_OFF();
    BUZZER_OFF();

    /* 4. Khởi tạo mô hình TinyML Autoencoder */
    tinyml_init();

    /* 5. Khởi tạo ngoại vi GPS NEO-M6N trên UART0 (9600 baud, P3.1 TX, P3.2 RX) */
    GPS_Init();
    DEBUG_UART_SendString("[GPS] UART0 9600 OK\r\n");

    /* 6. Khởi tạo Module 4G LTE SIM A7680C trên UART1 (115200 baud, P2.2 TX, P2.3 RX) */
    DEBUG_UART_SendString("[MODEM] Initializing SIM A7680C (UART1 115200)...\r\n");
    SIM4G_Init();
    if (g_sim_ready) {
        DEBUG_UART_SendString("[MODEM] Ready (CPIN OK)\r\n");
    } else {
        DEBUG_UART_SendString("[MODEM] WARN: Not Ready or SIM Missing\r\n");
    }

    /* Đánh dấu ngoại vi đã sẵn sàng để Delay_ms và Buzzer_Beep có thể Poll */
    g_periph_init_done = 1;

    BUZZER_OFF();
    ALERT_LED_OFF();
    Restore_SW3_Matrix_GPIO();

    s_last_hb_ms = g_tick_ms;
    DEBUG_UART_SendString("[SYS] Ready. Listening for Button / Trigger events.\r\n");
    DEBUG_UART_SendString("========================================\r\n\r\n");

    while (1)
    {
        uint32_t now = g_tick_ms;

        /* A. Duy trì cỗ máy trạng thái tách chuỗi NMEA từ GPS trên UART0 và đệm SIM 4G */
        GPS_Process();
        SIM4G_Poll();

        /* B. Nhịp tim hệ thống Heartbeat trên LED D6 (P3.8):
         *    - Khi chưa khóa vệ tinh GPS (Fix=0): Chớp tắt 500ms
         *    - Khi đã khóa vệ tinh GPS (Fix=1): Chớp tắt nhanh 200ms
         */
        uint32_t hb_period = g_gps.fix_valid ? 200UL : 500UL;
        if ((now - s_last_hb_ms) >= hb_period) {
            s_last_hb_ms = now;
            if (SN_GPIO3->DATA & (1UL << 8)) {
                LED_D6_ON();
            } else {
                LED_D6_OFF();
            }
            /* Đảm bảo chân P1.4 luôn giữ 0V mỗi nhịp tim để nút SW3 luôn nhạy */
            SN_GPIO1->BCLR = (1UL << 4);
        }

        /* Gửi nhịp tim UART định kỳ mỗi 2000ms để quan sát hệ thống trên máy tính */
        {
            static uint32_t s_last_uart_hb = 0;
            if ((now - s_last_uart_hb) >= 2000UL) {
                s_last_uart_hb = now;
                DEBUG_UART_Printf("[SYS] Heartbeat | Tick: %u ms | GPS Fix: %d (Sats: %u) | SIM Ready: %d\r\n",
                                  now, g_gps.fix_valid, g_gps.satellites, g_sim_ready);
            }
        }

        /* Tu dong ket noi lai SIM 4G moi 4 giay neu chua dong bo */
        if (!g_sim_ready) {
            static uint32_t s_last_sim_retry = 0;
            if ((now - s_last_sim_retry) >= 4000UL) {
                s_last_sim_retry = now;
                DEBUG_UART_SendString("[SYS] Dang ket noi lai SIM A7680C (UART1)...\r\n");
                SIM4G_Init();
            }
        }

        /* C. BẮT LỆNH ĐIỀU KHIỂN TỪ MÁY TÍNH QUA DEBUG UART (P3.11 RX) */
        if (DEBUG_UART_Available()) {
            char cmd = DEBUG_UART_ReadChar();
            if (cmd == 'c' || cmd == 'C') {
                DEBUG_UART_SendString("\r\n>>> [LENH PC] Kiem tra ket noi Module SIM A7680C... <<<\r\n");
                SIM4G_Init();
            } else if (cmd == 't' || cmd == 'T' || cmd == '1') {
                Execute_Emergency_Routine("PC Serial Command ('t')");
            } else if (cmd == 's' || cmd == 'S') {
                DEBUG_UART_Printf("[STATUS] Tick=%u ms, GPS Fix=%d, Sats=%u, Lat=%s, Lon=%s, SIM=%d\r\n",
                                  now, g_gps.fix_valid, g_gps.satellites,
                                  g_gps.latitude, g_gps.longitude, g_sim_ready);
            }
        }

        /* D. TỰ ĐỘNG KIỂM THỬ KHÔNG CẦN CHỜ NGƯỜI DÙNG BẤM NÚT (AUTONOMOUS TESTS) */
        {
            static uint32_t s_last_autotest_ms = 0;
            static uint8_t s_test_count = 0;

            /* Lần 1: Sau khi khởi động 10 giây (khi SIM đã sẵn sàng)
             * Lần 2, 3: Định kỳ mỗi 25 giây sau lần trước (tổng cộng 3 lần test độc lập)
             */
            if ((s_test_count < 3) && g_sim_ready &&
                ((s_test_count == 0 && now >= 10000UL) ||
                 (s_test_count > 0 && (now - s_last_autotest_ms) >= 25000UL)))
            {
                s_test_count++;
                s_last_autotest_ms = now;
                DEBUG_UART_SendString("\r\n========================================\r\n");
                DEBUG_UART_Printf(">>> [AUTO TEST #%d] Kich hoat gui SMS tu dong den %s <<<\r\n", s_test_count, EMERGENCY_PHONE_NUMBER);
                DEBUG_UART_SendString("========================================\r\n");
                Execute_Emergency_Routine("Autonomous Periodic Test");
            }
        }

        /* E. BẮT SỰ KIỆN NÚT BẤM TEST SW3 (P2.4) TRÊN BO MẠCH SONIX EVK:
         *    Nhấn bao nhiêu lần cũng kêu còi và gửi được tin nhắn ổn định 100%!
         */
        if ((SN_GPIO2->DATA & (1UL << 4)) == 0UL) {
            Delay_ms(25); /* Chống dội phím cơ học khi nhấn */
            if ((SN_GPIO2->DATA & (1UL << 4)) == 0UL) {
                DEBUG_UART_SendString("\r\n>>> [BUTTON] SW3 PRESSED (Emergency Trigger) <<<\r\n");

                /* Kích hoạt chu trình khẩn cấp: Bíp còi + Bật đèn + Gửi SMS */
                Execute_Emergency_Routine("SW3 Onboard Button");

                /* Đợi người dùng nhả phím SW3 ra hoàn toàn kèm timeout an toàn */
                uint32_t hold_start = GetTick_ms();
                while (((SN_GPIO2->DATA & (1UL << 4)) == 0UL) && ((GetTick_ms() - hold_start) < 2000UL)) {
                    GPS_Process();
                }
                Delay_ms(150); /* Chống dội khi nhả phím */
                Restore_SW3_Matrix_GPIO();
                DEBUG_UART_SendString("[BUTTON] SW3 Released. Ready for next press!\r\n");
            }
        }

        /* E. BẮT SỰ KIỆN XUNG WAKE-UP TỪ FPGA (Pin 41 -> P2.9 hoặc P2.8) */
        if ((SN_GPIO2->DATA & (1UL << 9)) || (SN_GPIO2->DATA & (1UL << 8))) {
            Delay_ms(25);
            if ((SN_GPIO2->DATA & (1UL << 9)) || (SN_GPIO2->DATA & (1UL << 8))) {
                DEBUG_UART_SendString("\r\n>>> [FPGA] Wake-up IRQ detected (Pin 41 -> P2.9/P2.8) <<<\r\n");
                Execute_Emergency_Routine("FPGA Wake-up Trigger (Pin 41)");

                uint32_t pulse_start = GetTick_ms();
                while (((SN_GPIO2->DATA & (1UL << 9)) || (SN_GPIO2->DATA & (1UL << 8))) &&
                       ((GetTick_ms() - pulse_start) < 2000UL)) {
                    GPS_Process();
                }
                Delay_ms(150);
                Restore_SW3_Matrix_GPIO();
            }
        }

        /* F. KIỂM TRA GÓI TIN 4 ĐẶC TRƯNG SPI TỪ FPGA GỬI SANG (BỘ LỌC TINH TINYML) */
        if (g_spi_pkt_ready) {
            g_spi_pkt_ready = 0;

            tinyml_features_t feat;
            feat.pitch    = (float)g_features[0] / 256.0f;
            feat.roll     = (float)g_features[1] / 256.0f;
            feat.rms      = (float)g_features[2] / 256.0f;
            feat.max_jerk = (float)g_features[3] / 16.0f;

            float mse_score = tinyml_predict(&feat);
            if (tinyml_confirm_accident(feat.roll, mse_score, 50)) {
                if (((now - s_last_sms_trigger_ms) >= 5000UL) || (s_last_sms_trigger_ms == 0UL)) {
                    s_last_sms_trigger_ms = now;
                    Execute_Emergency_Routine("TinyML Accident Detection");
                }
            }
        }
    }
}