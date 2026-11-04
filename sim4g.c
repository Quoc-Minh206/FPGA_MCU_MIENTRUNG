/******************************************************************************
 * sim4g.c  -  Driver Bare-metal 4G LTE SIM A7680C cho SONiX SN32F407 EVK
 *
 * Kết nối phần cứng:
 *  - P2.2 (UTXD1 - Chân 2 Cụm J7) ---> RXD của SIM A7680C
 *  - P2.3 (URXD1 - Chân 3 Cụm J7) <--- TXD của SIM A7680C
 *  - GND của bo Sonix EVK         <---> GND của SIM A7680C & IP5306
 *
 * Thao tác thanh ghi Bare-metal:
 *  - SN_PFPA->UART1: Bits [3:2]=01b (URXD1=P2.3), Bits [1:0]=01b (UTXD1=P2.2)
 *  - SN_GPIO2: Cấu hình P2.2 Output High, P2.3 Input Pull-up
 *  - SN_SYS1->AHBCLKEN: Bit 17 = 1 (Clock UART1)
 *  - SN_SYS1->PRST: Bit 17 = 1 (Reset UART1)
 *  - SN_UART1: DLL=4, DLM=0, FD=0x75 -> 115200 bps @ 12MHz
 *****************************************************************************/
#include <SN32F400.h>
#include "sim4g.h"
#include "system_shared.h"
#include "debug_uart.h"

/*=============================  ĐỊNH NGHĨA BIT THANH GHI  =================*/
#define UART1_CLKEN_BIT     (1UL << 17)     /* SYS1_AHBCLKEN.UART1CLKEN     */
#define UART1_RST_BIT       (1UL << 17)     /* SYS1_PRST.UART1RST           */
#define PFPA_UART1_P22_P23  ((1UL << 2) | (1UL << 0))

/* Bit thanh ghi SN_UART1->LS (Line Status Register - Offset 0x20) */
#define UART_LS_RDR         (1UL << 0)      /* Receiver Data Ready          */
#define UART_LS_OE          (1UL << 1)      /* Overrun Error                */
#define UART_LS_THRE        (1UL << 5)      /* Transmitter Holding Empty    */
#define UART_LS_TEMT        (1UL << 6)      /* Transmitter Empty            */

extern uint32_t GetTick_ms(void);
extern void     Delay_ms(uint32_t ms);

/*=============================  BIẾN TOÀN CỤC  ============================*/
volatile char     g_sim_rx_buf[SIM_RX_BUFFER_SIZE];
volatile uint16_t g_sim_rx_len = 0;
volatile uint8_t  g_sim_ready = 0;
volatile uint8_t  g_sim_sms_sent = 0;

/*=============================  HÀM XỬ LÝ CHUỖI BARE-METAL  ===============*/
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

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (g_sim_rx_len < SIM_RX_BUFFER_SIZE) {
        g_sim_rx_buf[g_sim_rx_len] = '\0';
    } else {
        g_sim_rx_buf[SIM_RX_BUFFER_SIZE - 1U] = '\0';
    }
    if (u_strstr((const char *)g_sim_rx_buf, needle)) {
        found = 1;
    }
    if (!primask) {
        __enable_irq();
    }
    return found;
}

static uint8_t SIM4G_RxContainsChar(char c)
{
    uint8_t found = 0;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (g_sim_rx_len < SIM_RX_BUFFER_SIZE) {
        g_sim_rx_buf[g_sim_rx_len] = '\0';
    } else {
        g_sim_rx_buf[SIM_RX_BUFFER_SIZE - 1U] = '\0';
    }
    if (u_strchr((const char *)g_sim_rx_buf, c)) {
        found = 1;
    }
    if (!primask) {
        __enable_irq();
    }
    return found;
}

/*
 * Xóa sạch bộ đệm phần mềm g_sim_rx_buf và thanh ghi RX FIFO phần cứng một cách an toàn
 */
void SIM4G_ClearRx(void)
{
    volatile uint32_t dummy = SN_UART1->LS;
    int guard = 32;

    /* Đọc xả sạch toàn bộ byte tồn đọng trong RX FIFO */
    while ((SN_UART1->LS & UART_LS_RDR) && (--guard > 0)) {
        dummy = SN_UART1->RB;
    }
    (void)dummy;

    /* Xóa bộ đệm RAM */
    g_sim_rx_len = 0;
    g_sim_rx_buf[0] = '\0';
}

/*=============================  BỘ THU POLLED UART1 (CHỐNG TREO)  ==========*/
/*
 * Hàm thăm dò SIM4G_Poll():
 * Thu gom trực tiếp các byte phản hồi từ modem qua thanh ghi UART1_RB.
 * Chạy đồng bộ trong mọi vòng lặp chờ và trong Delay_ms, bảo đảm:
 * 1. Không bao giờ làm rơi rụng byte phản hồi từ modem.
 * 2. TUYỆT ĐỐI KHÔNG BỊ TREO CPU hay kẹt ISR như phương pháp dùng ngắt NVIC.
 * 3. Bộ đếm guard = 32 bảo đảm không bao giờ lặp vô tận.
 */
void SIM4G_Poll(void)
{
    volatile uint32_t ls = SN_UART1->LS;
    (void)ls;

    int guard = 32;
    while ((SN_UART1->LS & UART_LS_RDR) && (--guard > 0)) {
        uint8_t b = (uint8_t)(SN_UART1->RB & 0xFFUL);
        if (b != 0x00U && g_sim_rx_len < (SIM_RX_BUFFER_SIZE - 2U)) {
            g_sim_rx_buf[g_sim_rx_len++] = (char)b;
            g_sim_rx_buf[g_sim_rx_len] = '\0';
        }
    }
}

void UART1_IRQHandler(void)
{
    /* Vô hiệu hóa ngắt nếu vô tình phát sinh để bảo vệ CPU */
    SN_UART1->IE = 0x00UL;
    NVIC_DisableIRQ(UART1_IRQn);
    NVIC_ClearPendingIRQ(UART1_IRQn);
}

/*=============================  TRUYỀN DỮ LIỆU UART1  =====================*/
void SIM4G_SendByte(uint8_t b)
{
    uint32_t timeout = 100000UL;
    /* Đợi thanh ghi giữ phát trống (THRE - Bit 5 của LS) */
    while (((SN_UART1->LS & UART_LS_THRE) == 0UL) && (--timeout > 0UL));
    SN_UART1->TH = (uint32_t)b;
}

void SIM4G_SendStr(const char *str)
{
    if (!str) return;
    while (*str) {
        SIM4G_SendByte((uint8_t)*str++);
    }
    /* Đợi truyền xong hoàn toàn byte cuối cùng ra khỏi thanh ghi dịch */
    uint32_t timeout = 100000UL;
    while (((SN_UART1->LS & UART_LS_TEMT) == 0UL) && (--timeout > 0UL));
}

uint8_t SIM4G_SendCmd(const char *cmd, const char *expected, uint32_t timeout_ms)
{
    uint32_t t_start;

    SIM4G_ClearRx();
    SIM4G_SendStr(cmd);

    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < timeout_ms)
    {
        SIM4G_Poll();
        if (expected && SIM4G_RxContains(expected)) {
            return 1;
        }
        if (SIM4G_RxContains("ERROR")) {
            return 0;
        }
        Delay_ms(2);
    }
    return 0;
}

/*=============================  KHỞI TẠO PHẦN CỨNG UART1 & SIM  ===========*/
static void SIM4G_Hardware_UART1_Init(void)
{
    /* 1. Cấu hình điện học cho chân GPIO P2.2 (UTXD1) và P2.3 (URXD1) */
    SN_GPIO2->BSET |= (1UL << 2);
    SN_GPIO2->MODE  = (SN_GPIO2->MODE | (1UL << 2)) & ~(1UL << 3);
    SN_GPIO2->CFG  &= ~((3UL << 4) | (3UL << 6)); /* Pull-up cho P2.2 và P2.3 */

    /* 2. Ánh xạ chân ngoại vi PFPA_UART1: URXD1 = P2.3 (01b), UTXD1 = P2.2 (01b) */
    SN_PFPA->UART1 = (SN_PFPA->UART1 & ~0x0FUL) | PFPA_UART1_P22_P23;

    /* 3. Cấp xung nhịp AHB cho UART1 và Reset cứng ngoại vi UART1 */
    SN_SYS1->AHBCLKEN |= UART1_CLKEN_BIT;
    SN_SYS1->PRST     |= UART1_RST_BIT;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* 4. Cấu hình Baudrate 115200 bps @ PCLK = 12.0 MHz */
    SN_UART1->LC  = 0x83UL;
    SN_UART1->FD  = (1UL << 7) | (11UL << 4) | (7UL << 0); /* FD = 0x75 */
    SN_UART1->DLM = 0UL;
    SN_UART1->DLL = 4UL;
    SN_UART1->LC  = 0x03UL; /* Khóa DLAB = 0 */

    /* 5. Kích hoạt FIFO, xóa sạch TX/RX FIFO, đặt RX Trigger Level = 1 character */
    SN_UART1->FIFOCTRL = 0x07UL;

    /* 6. Vô hiệu hóa ngắt phần cứng UART1 để sử dụng cơ chế Polling an toàn 100% */
    SN_UART1->IE = 0x00UL;
    NVIC_DisableIRQ(UART1_IRQn);
    NVIC_ClearPendingIRQ(UART1_IRQn);

    /* 7. Bật UART1 (Bit 0 = UARTEN, Bit 6 = RXEN, Bit 7 = TXEN) */
    SN_UART1->CTRL = 0xC1UL;
}

static void SIM4G_ExecAndPrint(const char *label, const char *cmd, uint32_t wait_ms)
{
    SIM4G_ClearRx();
    SIM4G_SendStr(cmd);
    uint32_t t0 = GetTick_ms();
    while ((GetTick_ms() - t0) < wait_ms) {
        SIM4G_Poll();
        Delay_ms(5);
    }
    DEBUG_UART_Printf("[%s] %s -> %s\r\n", label, cmd, (const char *)g_sim_rx_buf);
}

void SIM4G_Init(void)
{
    SIM4G_Hardware_UART1_Init();
    Delay_ms(400);
    SIM4G_ClearRx();

    /* Giải phóng SIM A7680C nếu trước đó đang bị kẹt trong chế độ chờ '>' */
    SIM4G_SendStr("\x1B\r\n");
    Delay_ms(150);
    SIM4G_ClearRx();

    DEBUG_UART_SendString("[MODEM] Dang dong bo baudrate AT (115200 8N1)...\r\n");
    uint8_t at_ok = 0;
    SIM4G_SendStr("\r\n");
    Delay_ms(50);
    for (uint8_t i = 0; i < 16; i++) {
        if (SIM4G_SendCmd("AT\r\n", "OK", 250)) {
            at_ok = 1;
            break;
        }
        SIM4G_SendStr("\x1B\r\n");
        Delay_ms(100);
    }

    if (!at_ok) {
        DEBUG_UART_SendString("[MODEM] LOI: Khong nhan duoc phan hoi AT tu SIM A7680C!\r\n");
        DEBUG_UART_SendString("  -> Kiem tra TXD SIM vao P2.3 (MCU), RXD SIM vao P2.2 (MCU)\r\n");
        DEBUG_UART_SendString("  -> Kiem tra GND chung giua SIM va MCU Sonix EVK\r\n");
        DEBUG_UART_SendString("  -> Kiem tra nguon VBAT/VCC (3.8V - 4.2V, 2A) cho module SIM\r\n");
        g_sim_ready = 0;
        return;
    }

    DEBUG_UART_SendString("[MODEM] KET NOI THANH CONG! (Phan hoi: AT -> OK)\r\n");
    SIM4G_SendCmd("ATE0\r\n", "OK", 300);            /* Echo Off */
    SIM4G_SendCmd("AT+CMEE=2\r\n", "OK", 300);       /* Verbose Error */
    SIM4G_SendCmd("AT+CMGF=1\r\n", "OK", 400);       /* SMS Text Mode */

    /* Kiểm tra thẻ SIM */
    if (!SIM4G_SendCmd("AT+CPIN?\r\n", "READY", 600)) {
        DEBUG_UART_SendString("[MODEM] CANH BAO: The SIM chua san sang (+CPIN != READY)\r\n");
        g_sim_ready = 0;
        return;
    }
    DEBUG_UART_SendString("[MODEM] The SIM: READY\r\n");

    /* Kiểm tra chất lượng sóng và nhà mạng */
        /* IN TOAN BO THONG TIN CHAN DOAN MODEM RA DEBUG UART */
    SIM4G_ExecAndPrint("MODEM-INFO", "ATI\r\n", 300);
    SIM4G_ExecAndPrint("MODEM-IMSI", "AT+CIMI\r\n", 300);
    SIM4G_ExecAndPrint("MODEM-CSQ",  "AT+CSQ\r\n", 300);
    SIM4G_ExecAndPrint("MODEM-COPS", "AT+COPS?\r\n", 500);
    SIM4G_ExecAndPrint("MODEM-SMSC", "AT+CSCA?\r\n", 500);
    SIM4G_ExecAndPrint("MODEM-CREG", "AT+CREG?\r\n", 300);
    SIM4G_ExecAndPrint("MODEM-CEREG","AT+CEREG?\r\n", 300);
    SIM4G_ExecAndPrint("MODEM-INBOX","AT+CMGL=\"ALL\"\r\n", 800);
    SIM4G_ExecAndPrint("MODEM-CNUM", "AT+CNUM\r\n", 400);
    SIM4G_ExecAndPrint("MODEM-MSG1", "AT+CMGR=1\r\n", 600);
    SIM4G_ExecAndPrint("MODEM-MSG2", "AT+CMGR=2\r\n", 600);
    SIM4G_ExecAndPrint("MODEM-BAL",  "AT+CUSD=1,\"*101#\",15\r\n", 1500);

    DEBUG_UART_SendString("[MODEM] Dang kiem tra dang ky mang...\r\n");

    /* Chờ đăng ký mạng viễn thông */
    for (uint8_t k = 0; k < 10; k++) {
        if (SIM4G_SendCmd("AT+CREG?\r\n", ",1", 300) ||
            SIM4G_RxContains(",5") ||
            SIM4G_SendCmd("AT+CEREG?\r\n", ",1", 300) ||
            SIM4G_RxContains(",5")) {
            DEBUG_UART_SendString("[MODEM] Da dang ky mang di dong thanh cong (Home/Roaming)!\r\n");
            g_sim_ready = 1;
            return;
        }
        Delay_ms(300);
    }

    DEBUG_UART_SendString("[MODEM] Dang tim kiem tram BTS mang, san sang khi co tin hieu.\r\n");
    g_sim_ready = 1;
}

/*=============================  GỬI TIN NHẮN SMS CỨU HỘ  ==================*/
static uint8_t SIM4G_SendSMS_Internal(const char *phone_number, const char *message)
{
    uint32_t t_start;
    char cmd[48];
    uint32_t i, p_len;
    uint8_t got_prompt = 0;

    /* 1. Reset trạng thái dở dang của modem bằng ESC */
    SIM4G_SendStr("\x1B\r\n");
    Delay_ms(100);
    SIM4G_ClearRx();

    /* 2. Kiểm tra đồng bộ AT với SIM A7680C */
    uint8_t synced = 0;
    for (uint8_t retry = 0; retry < 4; retry++) {
        if (SIM4G_SendCmd("AT\r\n", "OK", 300)) {
            synced = 1;
            break;
        }
        SIM4G_SendStr("\x1B\r\n");
        Delay_ms(100);
    }
    if (!synced) {
        DEBUG_UART_SendString("[SMS] MODEM ERROR: No AT response\r\n");
        g_sim_ready = 0;
        return 0;
    }

    /* 3. Thiết lập lại cấu hình SMS Text Mode trước MỖI lần gửi */
    SIM4G_SendCmd("ATE0\r\n", "OK", 250);
    SIM4G_SendCmd("AT+CMGF=1\r\n", "OK", 350);

    /* 4. Đóng gói lệnh: AT+CMGS="<phone_number>"\r */
    cmd[0] = 'A'; cmd[1] = 'T'; cmd[2] = '+'; cmd[3] = 'C';
    cmd[4] = 'M'; cmd[5] = 'G'; cmd[6] = 'S'; cmd[7] = '=';
    cmd[8] = '\"';
    i = 9;
    p_len = u_strlen(phone_number);
    if (p_len > 24U) p_len = 24U;
    for (uint32_t j = 0; j < p_len; j++) {
        cmd[i++] = phone_number[j];
    }
    cmd[i++] = '\"';
    cmd[i++] = '\r';
    cmd[i++] = '\n';
    cmd[i]   = '\0';

    DEBUG_UART_Printf("[SMS] Sending: AT+CMGS=\"%s\" -> Waiting for '>'\r\n", phone_number);

    SIM4G_ClearRx();
    SIM4G_SendStr(cmd);

    /* 5. Chờ dấu nhắc '>' từ SIM A7680C (tối đa 5000 ms) */
    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < 5000UL)
    {
        SIM4G_Poll();
        if (SIM4G_RxContainsChar('>')) {
            got_prompt = 1;
            break;
        }
        if (SIM4G_RxContains("ERROR")) {
            DEBUG_UART_Printf("[SMS] CMGS ERROR Response: %s\r\n", (const char *)g_sim_rx_buf);
            break;
        }
        Delay_ms(5);
    }

    if (!got_prompt) {
        DEBUG_UART_Printf("[SMS] TIMEOUT waiting for '>'. RX: %s\r\n", (const char *)g_sim_rx_buf);
        SIM4G_SendStr("\x1B\r\n");
        Delay_ms(100);
        SIM4G_ClearRx();
        return 0;
    }

    Delay_ms(50);

    /* 6. Xóa bộ đệm RX trước khi bơm nội dung và Ctrl+Z (0x1A) */
    SIM4G_ClearRx();
    SIM4G_SendStr(message);
    Delay_ms(50);
    SIM4G_SendByte(0x1A);

    DEBUG_UART_SendString("[SMS] Body + Ctrl+Z sent. Waiting for +CMGS network ACK...\r\n");

    /* 7. Chờ phản hồi xác nhận từ nhà mạng: Chấp nhận +CMGS: hoặc OK */
    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < 20000UL)
    {
        SIM4G_Poll();
        if (SIM4G_RxContains("+CMGS:") || SIM4G_RxContains("OK"))
        {
            Delay_ms(100);
            DEBUG_UART_Printf("[SMS] SUCCESS! Modem ACK: %s\r\n", (const char *)g_sim_rx_buf);
            g_sim_sms_sent = 1;
            g_sim_ready = 1;
            SIM4G_ClearRx();
            return 1;
        }

        if (SIM4G_RxContains("+CMS ERROR") || SIM4G_RxContains("ERROR"))
        {
            Delay_ms(50);
            DEBUG_UART_Printf("[SMS] NETWORK ERROR: %s\r\n", (const char *)g_sim_rx_buf);
            SIM4G_SendStr("\x1B\r\n");
            Delay_ms(100);
            SIM4G_ClearRx();
            return 0;
        }
        Delay_ms(20);
    }

    DEBUG_UART_Printf("[SMS] TIMEOUT (20s). RX: %s\r\n", (const char *)g_sim_rx_buf);
    SIM4G_SendStr("\x1B\r\n");
    Delay_ms(100);
    SIM4G_ClearRx();
    return 0;
}

uint8_t SIM4G_SendSMS(const char *phone_number, const char *message)
{
    if (!phone_number || !message) return 0;

    /* Thử gửi với định dạng số ban đầu (0932591161) */
    uint8_t ok = SIM4G_SendSMS_Internal(phone_number, message);

    /* Nếu thất bại và số bắt đầu bằng 0, tự động thử lại bằng định dạng quốc tế +84 */
    if (!ok && phone_number[0] == '0')
    {
        DEBUG_UART_SendString("[SMS] RETRY: Switching to +84 international format...\r\n");
        char intl_num[32];
        intl_num[0] = '+';
        intl_num[1] = '8';
        intl_num[2] = '4';
        uint32_t i = 3;
        for (uint32_t j = 1; phone_number[j] != '\0' && i < 30U; j++) {
            intl_num[i++] = phone_number[j];
        }
        intl_num[i] = '\0';

        Delay_ms(1000);
        ok = SIM4G_SendSMS_Internal(intl_num, message);
    }
    /* Ngược lại nếu số bắt đầu bằng +84 và thất bại, thử lại bằng số nội địa 0x */
    else if (!ok && phone_number[0] == '+' && phone_number[1] == '8' && phone_number[2] == '4')
    {
        DEBUG_UART_SendString("[SMS] RETRY: Switching to domestic 0x format...\r\n");
        char local_num[32];
        local_num[0] = '0';
        uint32_t i = 1;
        for (uint32_t j = 3; phone_number[j] != '\0' && i < 30U; j++) {
            local_num[i++] = phone_number[j];
        }
        local_num[i] = '\0';

        Delay_ms(1000);
        ok = SIM4G_SendSMS_Internal(local_num, message);
    }
    return ok;
}
