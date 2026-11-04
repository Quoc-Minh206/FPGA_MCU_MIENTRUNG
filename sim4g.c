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
    uint32_t ii = SN_UART1->II;
    uint32_t ls;

    while ((ii & 0x01UL) == 0UL)
    {
        switch ((ii >> 1) & 0x07UL)
        {
        case 3: /* RLS */
            ls = SN_UART1->LS;
            (void)ls;
            (void)SN_UART1->RB;
            break;

        case 2: /* RDA */
        case 6: /* CTO */
            while (SN_UART1->LS & 0x01UL)
            {
                uint8_t b = (uint8_t)(SN_UART1->RB & 0xFFUL);
                if (g_sim_rx_len < (SIM_RX_BUFFER_SIZE - 1))
                {
                    g_sim_rx_buf[g_sim_rx_len++] = (char)b;
                    g_sim_rx_buf[g_sim_rx_len] = '\0';
                }
            }
            break;

        default:
            (void)SN_UART1->LS;
            (void)SN_UART1->RB;
            break;
        }
        ii = SN_UART1->II;
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
        DEBUG_UART_SendString("[MODEM] Text mode configured\r\n");

        if (SIM4G_SendCmd("AT+CPIN?\r\n", "READY", 500)) {
            DEBUG_UART_SendString("[MODEM] SIM READY\r\n");
            g_sim_ready = 1;
        } else {
            DEBUG_UART_SendString("[MODEM] SIM Not Ready\r\n");
            g_sim_ready = 0;
        }
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
    char cmd[48];
    uint8_t ctrl_z = 0x1A;
    uint32_t i, p_len;
    uint8_t got_prompt = 0;

    /* 1. Giải phóng modem khỏi trạng thái treo (nếu có) bằng mã ESC (0x1B) */
    SIM4G_SendByte(0x1B);
    Delay_ms(50);
    SIM4G_ClearRx();

    /* Kiểm tra nhanh kết nối AT với modem */
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

    /* 2. Tạo lệnh AT+CMGS="<phone_number>"\r */
    cmd[0] = 'A'; cmd[1] = 'T'; cmd[2] = '+'; cmd[3] = 'C';
    cmd[4] = 'M'; cmd[5] = 'G'; cmd[6] = 'S'; cmd[7] = '=';
    cmd[8] = '\"';
    i = 9;
    p_len = u_strlen(phone_number);
    if (p_len > 24) p_len = 24;
    for (uint32_t j = 0; j < p_len; j++) cmd[i++] = phone_number[j];
    cmd[i++] = '\"';
    cmd[i++] = '\r';
    cmd[i] = '\0';

    DEBUG_UART_Printf("[SMS] AT+CMGS=\"%s\"\r\n", phone_number);
    SIM4G_ClearRx();
    SIM4G_SendStr(cmd);

    /* 3. Chờ dấu nhắc '>' từ module SIM (tối đa 5 giây) */
    DEBUG_UART_SendString("[SMS] WAIT_PROMPT (>)\r\n");
    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < 5000)
    {
        if (SIM4G_RxContainsChar('>')) {
            got_prompt = 1;
            break;
        }
        if (SIM4G_RxContains("ERROR")) {
            DEBUG_UART_SendString("[SMS] ERROR: Modem returned ERROR on CMGS\r\n");
            break;
        }
        Delay_ms(10);
    }

    if (!got_prompt) {
        DEBUG_UART_SendString("[SMS] TIMEOUT waiting for '>'\r\n");
        SIM4G_SendByte(0x1B); /* ESC */
        Delay_ms(50);
        return 0;
    }

    DEBUG_UART_SendString("[SMS] SEND_DATA: Sending body + Ctrl+Z\r\n");
    Delay_ms(100);

    /* 4. Truyền nội dung tin nhắn và kết thúc bằng Ctrl+Z */
    SIM4G_ClearRx();
    SIM4G_SendStr(message);
    Delay_ms(150);
    SIM4G_SendByte(ctrl_z);

    /* 5. Chờ phản hồi +CMGS hoặc OK từ nhà mạng (tối đa 25 giây) */
    DEBUG_UART_SendString("[SMS] WAIT_RESULT: Waiting for network ACK...\r\n");
    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < 25000)
    {
        /* CHẤP NHẬN CẢ +CMGS VÀ OK để đảm bảo nhận diện chính xác mạng viễn thông */
        if (SIM4G_RxContains("+CMGS") || SIM4G_RxContains("OK"))
        {
            DEBUG_UART_SendString("[SMS] SUCCESS (+CMGS/OK received)\r\n");
            g_sim_sms_sent = 1;
            g_sim_ready = 1;
            Delay_ms(200);
            SIM4G_ClearRx();
            return 1;
        }

        if (SIM4G_RxContains("ERROR"))
        {
            DEBUG_UART_SendString("[SMS] ERROR: Network / Modem returned ERROR\r\n");
            SIM4G_SendByte(0x1B);
            Delay_ms(100);
            SIM4G_ClearRx();
            return 0;
        }
        Delay_ms(50);
    }

    DEBUG_UART_SendString("[SMS] TIMEOUT: No response from network\r\n");
    SIM4G_SendByte(0x1B);
    Delay_ms(100);
    SIM4G_ClearRx();
    return 0;
}

uint8_t SIM4G_SendSMS(const char *phone_number, const char *message)
{
    if (!phone_number || !message) return 0;

    uint8_t ok = SIM4G_SendSMS_Internal(phone_number, message);
    if (!ok && phone_number[0] == '0')
    {
        DEBUG_UART_SendString("[SMS] RETRY: Trying +84 international format...\r\n");
        char intl_num[32];
        intl_num[0] = '+'; intl_num[1] = '8'; intl_num[2] = '4';
        uint32_t i = 3;
        for (uint32_t j = 1; phone_number[j] && i < 30; j++) {
            intl_num[i++] = phone_number[j];
        }
        intl_num[i] = '\0';
        Delay_ms(500);
        ok = SIM4G_SendSMS_Internal(intl_num, message);
    }
    return ok;
}
