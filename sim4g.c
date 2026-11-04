/******************************************************************************
 * sim4g.c  -  Driver 4G LTE SIM A7680C cho SONiX SN32F407 EVK
 *
 *  - UART1: 115200 bps, 8N1
 *  - P2.2 (UTXD1) -> RXD SIM A7680C (Cụm J7 Chân 2 - Bảng 4 constrain.pdf)
 *  - P2.3 (URXD1) -> TXD SIM A7680C (Cụm J7 Chân 3 - Bảng 4 constrain.pdf)
 *  - Nguồn module SIM được cấp trực tiếp từ mạch nguồn IP5306 / Pin Li-ion.
 *****************************************************************************/
#include <SN32F400.h>
#include "sim4g.h"
#include "system_shared.h"
#include "debug_uart.h"

/*=============================  CẤU HÌNH UART1  ============================*/
#define UART1_CLKEN_BIT     (1UL << 17)     /* SYS1_AHBCLKEN.UART1CLKEN     */

/* PFPA_UART1: [3:2] URXD1 = 1 -> P2.3 ; [1:0] UTXD1 = 1 -> P2.2            */
#define PFPA_UART1_P22_P23  ((1UL << 2) | (1UL << 0))

extern uint32_t GetTick_ms(void);
extern void     Delay_ms(uint32_t ms);

/*=============================  BIẾN TOÀN CỤC  ============================*/
volatile char     g_sim_rx_buf[SIM_RX_BUFFER_SIZE];
volatile uint16_t g_sim_rx_len = 0;
volatile uint8_t  g_sim_ready = 0;
volatile uint8_t  g_sim_sms_sent = 0;

/*=============================  HÀM TIỆN ÍCH  =============================*/
static uint32_t u_strlen(const char *s)
{
    uint32_t len = 0;
    while (s && s[len]) len++;
    return len;
}

static const char *u_strstr(const char *haystack, const char *needle)
{
    if (!haystack || !needle || !*needle) return haystack;
    for (; *haystack; haystack++) {
        const char *h = haystack;
        const char *n = needle;
        while (*h && *n && (*h == *n)) { h++; n++; }
        if (!*n) return haystack;
    }
    return 0;
}

static const char *u_strchr(const char *s, char c)
{
    if (!s) return 0;
    while (*s) {
        if (*s == c) return s;
        s++;
    }
    return 0;
}

static uint8_t SIM4G_RxContains(const char *needle)
{
    uint8_t found = 0;
    if (!needle) return 0;
    __disable_irq();
    if (g_sim_rx_len < SIM_RX_BUFFER_SIZE) {
        g_sim_rx_buf[g_sim_rx_len] = '\0';
    } else {
        g_sim_rx_buf[SIM_RX_BUFFER_SIZE - 1] = '\0';
    }
    if (u_strstr((const char *)g_sim_rx_buf, needle)) {
        found = 1;
    }
    __enable_irq();
    return found;
}

static uint8_t SIM4G_RxContainsChar(char c)
{
    uint8_t found = 0;
    __disable_irq();
    if (g_sim_rx_len < SIM_RX_BUFFER_SIZE) {
        g_sim_rx_buf[g_sim_rx_len] = '\0';
    } else {
        g_sim_rx_buf[SIM_RX_BUFFER_SIZE - 1] = '\0';
    }
    if (u_strchr((const char *)g_sim_rx_buf, c)) {
        found = 1;
    }
    __enable_irq();
    return found;
}

void SIM4G_ClearRx(void)
{
    __disable_irq();
    g_sim_rx_len = 0;
    g_sim_rx_buf[0] = '\0';
    __enable_irq();
}

/*=============================  NGẮT UART1  ===============================*/
void UART1_IRQHandler(void)
{
    uint32_t ls = SN_UART1->LS;

    /* Xóa cờ lỗi đường truyền nếu có */
    if (ls & 0x8EUL) {
        volatile uint32_t dummy = SN_UART1->RB;
        (void)dummy;
    }

    /* Đọc cạn các byte có sẵn trong RX FIFO */
    while (SN_UART1->LS & 0x01UL)
    {
        uint8_t b = (uint8_t)(SN_UART1->RB & 0xFFUL);
        if (g_sim_rx_len < (SIM_RX_BUFFER_SIZE - 1))
        {
            g_sim_rx_buf[g_sim_rx_len++] = (char)b;
            g_sim_rx_buf[g_sim_rx_len] = '\0';
        }
    }
}

/*=============================  KHỞI TẠO  =================================*/
void SIM4G_Init(void)
{
    SIM4G_ClearRx();

    /* 1. Định tuyến chân P2.2 (UTXD1), P2.3 (URXD1) ra cọc J7 */
    SN_PFPA->UART1 = (SN_PFPA->UART1 & ~0x0FUL) | PFPA_UART1_P22_P23;

    /* 2. Cấp xung nhịp cho khối UART1 */
    SN_SYS1->AHBCLKEN |= UART1_CLKEN_BIT;

    /* 3. Cấu hình Baudrate 115200 bps (IHRC 12 MHz) */
    SN_UART1->LC  = 0x03UL | 0x80UL;
    SN_UART1->FD  = 0x75UL;
    SN_UART1->DLM = 0;
    SN_UART1->DLL = 4;
    SN_UART1->LC  = 0x03UL;

    /* 4. FIFO: Reset và kích hoạt */
    SN_UART1->FIFOCTRL = 0x01UL | 0x02UL | 0x04UL;

    /* 5. Kích hoạt ngắt nhận RDA & RLS */
    SN_UART1->IE = 0x01UL | 0x04UL;
    NVIC_EnableIRQ(UART1_IRQn);

    /* 6. Bật bộ phát và bộ thu UART1 */
    SN_UART1->CTRL = 0x01UL | (1UL << 6) | (1UL << 7);

    /* 7. Đồng bộ tốc độ truyền với module A7680C */
    DEBUG_UART_SendString("[MODEM] Syncing AT baud...\r\n");
    uint8_t at_ok = 0;
    for (uint8_t i = 0; i < 5; i++) {
        if (SIM4G_SendCmd("AT\r\n", "OK", 300)) {
            at_ok = 1;
            break;
        }
        Delay_ms(100);
    }

    if (at_ok) {
        DEBUG_UART_SendString("[MODEM] AT -> OK\r\n");
        SIM4G_SendCmd("ATE0\r\n", "OK", 300);
        SIM4G_SendCmd("AT+CMGF=1\r\n", "OK", 300);
        SIM4G_SendCmd("AT+CSCS=\"GSM\"\r\n", "OK", 300);
        DEBUG_UART_SendString("[MODEM] Text mode configured (CMGF=1, CSCS=GSM)\r\n");

        if (SIM4G_SendCmd("AT+CPIN?\r\n", "READY", 500)) {
            DEBUG_UART_SendString("[MODEM] SIM Card: READY (CPIN OK)\r\n");
        } else {
            DEBUG_UART_SendString("[MODEM] WARN: SIM Card Missing or Locked\r\n");
        }

        /* Kiểm tra đăng ký mạng viễn thông di động (AT+CREG? trả về ,1 hoặc ,5) */
        DEBUG_UART_SendString("[MODEM] Checking network registration (AT+CREG?)...\r\n");
        uint8_t net_registered = 0;
        for (uint8_t r = 0; r < 8; r++) {
            SIM4G_ClearRx();
            SIM4G_SendStr("AT+CREG?\r\n");
            Delay_ms(300);
            if (SIM4G_RxContains(",1") || SIM4G_RxContains(",5")) {
                net_registered = 1;
                break;
            }
            Delay_ms(200);
        }

        if (net_registered) {
            DEBUG_UART_SendString("[MODEM] Network: REGISTERED (Home/Roaming OK)\r\n");
            g_sim_ready = 1;
        } else {
            DEBUG_UART_SendString("[MODEM] WARN: Network Searching or No Signal (CREG not 1/5)\r\n");
            g_sim_ready = 0;
        }

        /* Đọc chất lượng sóng di động */
        SIM4G_ClearRx();
        SIM4G_SendStr("AT+CSQ\r\n");
        Delay_ms(200);
        DEBUG_UART_Printf("[MODEM] Signal Quality: %s\r\n", (const char *)g_sim_rx_buf);
    } else {
        DEBUG_UART_SendString("[MODEM] ERROR: No response to AT\r\n");
        g_sim_ready = 0;
    }
}

void SIM4G_SendByte(uint8_t b)
{
    uint32_t timeout = 50000;
    while (((SN_UART1->LS & (1UL << 5)) == 0) && (--timeout));
    SN_UART1->TH = b;
}

void SIM4G_SendStr(const char *str)
{
    if (!str) return;
    while (*str) {
        SIM4G_SendByte((uint8_t)*str++);
    }
}

uint8_t SIM4G_SendCmd(const char *cmd, const char *expected, uint32_t timeout_ms)
{
    uint32_t t_start;

    SIM4G_ClearRx();
    SIM4G_SendStr(cmd);

    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < timeout_ms)
    {
        if (expected && SIM4G_RxContains(expected)) {
            return 1;
        }
        if (SIM4G_RxContains("ERROR")) {
            return 0;
        }
        Delay_ms(10);
    }
    return 0;
}

/*=============================  GỬI TIN NHẮN SMS  =========================*/
static uint8_t SIM4G_SendSMS_Internal(const char *phone_number, const char *message)
{
    uint32_t t_start;
    char target_number[32];
    char cmd[48];
    uint8_t ctrl_z = 0x1A;
    uint32_t i, p_len;
    uint8_t got_prompt = 0;

    /* 1. Chuẩn hóa số điện thoại: Tự động đổi đầu 0 sang mã quốc tế +84 */
    if (phone_number[0] == '0') {
        target_number[0] = '+'; target_number[1] = '8'; target_number[2] = '4';
        uint32_t k = 3;
        for (uint32_t m = 1; phone_number[m] && k < 30; m++) {
            target_number[k++] = phone_number[m];
        }
        target_number[k] = '\0';
    } else {
        uint32_t k = 0;
        for (uint32_t m = 0; phone_number[m] && k < 30; m++) {
            target_number[k++] = phone_number[m];
        }
        target_number[k] = '\0';
    }

    /* 2. Giải phóng modem khỏi trạng thái treo (nếu có) bằng mã ESC (0x1B) */
    SIM4G_SendByte(0x1B);
    Delay_ms(50);
    SIM4G_ClearRx();

    /* Kiểm tra kết nối AT */
    if (!SIM4G_SendCmd("AT\r\n", "OK", 300)) {
        if (!SIM4G_SendCmd("AT\r\n", "OK", 300)) {
            DEBUG_UART_SendString("[SMS] MODEM ERROR: No AT response\r\n");
            g_sim_ready = 0;
            return 0;
        }
    }

    SIM4G_SendCmd("ATE0\r\n", "OK", 200);
    SIM4G_SendCmd("AT+CMGF=1\r\n", "OK", 300);
    SIM4G_SendCmd("AT+CSCS=\"GSM\"\r\n", "OK", 300);

    /* 3. Tạo lệnh AT+CMGS="<target_number>"\r */
    cmd[0] = 'A'; cmd[1] = 'T'; cmd[2] = '+'; cmd[3] = 'C';
    cmd[4] = 'M'; cmd[5] = 'G'; cmd[6] = 'S'; cmd[7] = '=';
    cmd[8] = '\"';
    i = 9;
    p_len = u_strlen(target_number);
    if (p_len > 24) p_len = 24;
    for (uint32_t j = 0; j < p_len; j++) cmd[i++] = target_number[j];
    cmd[i++] = '\"';
    cmd[i++] = '\r';
    cmd[i] = '\0';

    DEBUG_UART_Printf("[SMS] Sending AT+CMGS=\"%s\"\r\n", target_number);
    SIM4G_ClearRx();
    SIM4G_SendStr(cmd);

    /* 4. Chờ dấu nhắc '>' từ module SIM (tối đa 5 giây) */
    DEBUG_UART_SendString("[SMS] Waiting for prompt '> ' from A7680C...\r\n");
    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < 5000)
    {
        if (SIM4G_RxContainsChar('>')) {
            got_prompt = 1;
            break;
        }
        if (SIM4G_RxContains("ERROR")) {
            DEBUG_UART_Printf("[SMS] ERROR: Modem returned ERROR on CMGS: %s\r\n", (const char *)g_sim_rx_buf);
            break;
        }
        Delay_ms(10);
    }

    if (!got_prompt) {
        DEBUG_UART_Printf("[SMS] TIMEOUT waiting for '>'. Last response: %s\r\n", (const char *)g_sim_rx_buf);
        SIM4G_SendByte(0x1B); /* Gửi ESC để hủy */
        Delay_ms(50);
        return 0;
    }

    DEBUG_UART_SendString("[SMS] Prompt '>' received! Sending payload + Ctrl+Z...\r\n");
    Delay_ms(50);

    /* 5. Truyền nội dung tin nhắn và kết thúc bằng Ctrl+Z (0x1A) */
    SIM4G_ClearRx();
    SIM4G_SendStr(message);
    Delay_ms(100);
    SIM4G_SendByte(ctrl_z);

    /* 6. Chờ phản hồi xác nhận thực tế từ nhà mạng qua +CMGS: (tối đa 25 giây) */
    DEBUG_UART_SendString("[SMS] Waiting for Network ACK (+CMGS:)...\r\n");
    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < 25000)
    {
        /* CHỈ CHẤP NHẬN "+CMGS:" (có dấu hai chấm) để chống báo PASS giả từ echo AT+CMGS */
        if (SIM4G_RxContains("+CMGS:"))
        {
            Delay_ms(100);
            DEBUG_UART_Printf("[MODEM RX]: %s\r\n", (const char *)g_sim_rx_buf);
            DEBUG_UART_SendString("[SMS] SUCCESS! Network confirmed SMS dispatch (+CMGS: received)\r\n");
            g_sim_sms_sent = 1;
            g_sim_ready = 1;
            Delay_ms(100);
            SIM4G_ClearRx();
            return 1;
        }

        if (SIM4G_RxContains("ERROR") || SIM4G_RxContains("+CMS ERROR"))
        {
            Delay_ms(100);
            DEBUG_UART_Printf("[MODEM RX ERROR]: %s\r\n", (const char *)g_sim_rx_buf);
            DEBUG_UART_SendString("[SMS] FAILED: Cellular network or SIM rejected SMS!\r\n");
            SIM4G_SendByte(0x1B);
            Delay_ms(50);
            SIM4G_ClearRx();
            return 0;
        }
        Delay_ms(50);
    }

    DEBUG_UART_Printf("[SMS] TIMEOUT: No response from network within 25s. RX: %s\r\n", (const char *)g_sim_rx_buf);
    SIM4G_SendByte(0x1B);
    Delay_ms(50);
    SIM4G_ClearRx();
    return 0;
}

uint8_t SIM4G_SendSMS(const char *phone_number, const char *message)
{
    if (!phone_number || !message) return 0;
    return SIM4G_SendSMS_Internal(phone_number, message);
}
