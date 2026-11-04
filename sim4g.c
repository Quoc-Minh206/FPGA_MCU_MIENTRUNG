/******************************************************************************
 * sim4g.c  -  Driver 4G LTE SIM A7680C / SIM800 cho SONiX SN32F407 EVK
 *
 *  - UART1 115200 8N1, chan P2.2 (UTXD1) / P2.3 (URXD1) -> cum J7 tren EVK
 *  - Chan Reset / Power: P2.1
 *****************************************************************************/
#include <SN32F400.h>
#include "sim4g.h"
#include "system_shared.h"

/*=============================  CAU HINH  =================================*/
#define UART1_CLKEN_BIT     (1UL << 17)     /* SYS1_AHBCLKEN.UART1CLKEN     */

/* PFPA_UART1: [3:2] URXD1 = 1 -> P2.3 ; [1:0] UTXD1 = 1 -> P2.2            */
#define PFPA_UART1_P22_P23  ((1UL << 2) | (1UL << 0))

extern uint32_t GetTick_ms(void);
extern void     Delay_ms(uint32_t ms);

/*=============================  BIEN TOAN CUC  ============================*/
volatile char     g_sim_rx_buf[SIM_RX_BUFFER_SIZE];
volatile uint16_t g_sim_rx_len = 0;
volatile uint8_t  g_sim_ready = 0;
volatile uint8_t  g_sim_sms_sent = 0;

/*=============================  HAM TIEN ICH  =============================*/
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

void SIM4G_ClearRx(void)
{
    uint16_t i;
    for (i = 0; i < SIM_RX_BUFFER_SIZE; i++) g_sim_rx_buf[i] = 0;
    g_sim_rx_len = 0;
}

/*=============================  NGAT UART1  ===============================*/
void UART1_IRQHandler(void)
{
    uint32_t ii = SN_UART1->II;
    uint32_t ls;

    while ((ii & 0x01UL) == 0UL)            /* Co ngat dang cho             */
    {
        switch ((ii >> 1) & 0x07UL)
        {
        case 3:                             /* RLS: Loi duong truyen        */
            ls = SN_UART1->LS;
            (void)ls;
            (void)SN_UART1->RB;
            break;

        case 2:                             /* RDA: Co du lieu nhan         */
        case 6:                             /* Timeout ky tu                */
            while (SN_UART1->LS & 0x01UL)   /* RDR                          */
            {
                uint8_t b = (uint8_t)SN_UART1->RB;
                if (g_sim_rx_len < (SIM_RX_BUFFER_SIZE - 1))
                {
                    g_sim_rx_buf[g_sim_rx_len++] = (char)b;
                    g_sim_rx_buf[g_sim_rx_len] = '\0';
                }
            }
            break;

        default:
            (void)SN_UART1->LS;
            break;
        }
        ii = SN_UART1->II;
    }
}

/*=============================  KHOI TAO  =================================*/
void SIM4G_Init(void)
{
    SIM4G_ClearRx();

    /* 1. Dinh tuyen chan P2.2 (UTXD1), P2.3 (URXD1) */
    SN_PFPA->UART1 = (SN_PFPA->UART1 & ~0x0FUL) | PFPA_UART1_P22_P23;

    /* 2. Cap xung nhip cho UART1 */
    SN_SYS1->AHBCLKEN |= UART1_CLKEN_BIT;

    /* 3. Cau hinh Baud 115200 bps (IHRC 12 MHz):
     *    Oversampling 16, DLL = 4, DLM = 0, MULVAL = 7 (MUL = 8), DIVADDVAL = 5
     *    Baud = 12M / (16 * 4 * (1 + 5/8)) = 115,384 bps (sai so 0.16%)       */
    SN_UART1->LC  = 0x03UL | 0x80UL;        /* 8N1 + mo chot DLAB           */
    SN_UART1->FD  = 0x75UL;                 /* MULVAL[7:4]=7, DIVADDVAL[3:0]=5 */
    SN_UART1->DLM = 0;
    SN_UART1->DLL = 4;
    SN_UART1->LC  = 0x03UL;                 /* Dong chot DLAB               */

    /* 4. FIFO: Reset va kich hoat */
    SN_UART1->FIFOCTRL = 0x01UL | 0x02UL | 0x04UL;

    /* 5. Kich hoat ngat nhan RDA & RLS */
    SN_UART1->IE = 0x01UL | 0x04UL;
    NVIC_EnableIRQ(UART1_IRQn);

    /* 6. Bat bo phat va bo thu UART1 */
    SN_UART1->CTRL = 0x01UL | (1UL << 6) | (1UL << 7);

    /* 7. Giu chan P2.1 (PWRKEY/Reset) o muc HIGH (3.3V) mac dinh, khong lam tat nguon module */
    GPIO2_MODE |= (1UL << 1);
    GPIO2_BSET  = (1UL << 1);
    Delay_ms(500);

    /* 8. Kiem tra phan hoi AT */
    uint8_t at_ok = 0;
    for (uint8_t retry = 0; retry < 5; retry++) {
        if (SIM4G_SendCmd("AT\r\n", "OK", 500)) {
            at_ok = 1;
            break;
        }
        Delay_ms(150);
    }
    if (at_ok) {
        g_sim_ready = 1;
        SIM4G_SendCmd("ATE0\r\n", "OK", 500);
        SIM4G_SendCmd("AT+CMEE=1\r\n", "OK", 500);
        SIM4G_SendCmd("AT+CMGF=1\r\n", "OK", 500);
        SIM4G_SendCmd("AT+CSCS=\"GSM\"\r\n", "OK", 500);
        SIM4G_SendCmd("AT+CMGD=1,4\r\n", "OK", 1000);
    } else {
        g_sim_ready = 0;
    }
}

void SIM4G_SendByte(uint8_t b)
{
    while ((SN_UART1->LS & (1UL << 5)) == 0); /* Cho dem phat trong (THRE)  */
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
        if (expected && u_strstr((const char *)g_sim_rx_buf, expected)) {
            return 1;
        }
        if (u_strstr((const char *)g_sim_rx_buf, "ERROR")) {
            return 0;
        }
        Delay_ms(10);
    }
    return 0;
}

/*=============================  GUI SMS  ==================================*/
static uint8_t SIM4G_SendSMS_Internal(const char *phone_number, const char *message)
{
    uint32_t t_start;
    char cmd[48];
    uint8_t ctrl_z = 0x1A;
    uint32_t i, p_len;

    /* 1. Kiem tra module san sang (gui AT toi da 5 lan de dong bo baud) */
    uint8_t at_ok = 0;
    for (uint32_t k = 0; k < 5; k++) {
        if (SIM4G_SendCmd("AT\r\n", "OK", 500)) {
            at_ok = 1;
            break;
        }
        Delay_ms(150);
    }

    if (!at_ok) {
        return 0;
    }

    /* 2. Cau hinh Text Mode va bo ma GSM */
    SIM4G_SendCmd("ATE0\r\n", "OK", 500);
    SIM4G_SendCmd("AT+CPIN?\r\n", "READY", 1500);
    SIM4G_SendCmd("AT+CREG?\r\n", "OK", 1000);
    SIM4G_SendCmd("AT+CMGF=1\r\n", "OK", 1000);
    SIM4G_SendCmd("AT+CSCS=\"GSM\"\r\n", "OK", 1000);

    /* 3. Tao lenh AT+CMGS="<phone_number>"\r */
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

    SIM4G_ClearRx();
    SIM4G_SendStr(cmd);

    /* 4. Cho dau nhac '>' tu module SIM (toi da 5s) */
    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < 5000)
    {
        if (u_strchr((const char *)g_sim_rx_buf, '>')) break;
        if (u_strstr((const char *)g_sim_rx_buf, "ERROR")) return 0;
        Delay_ms(10);
    }
    if (!u_strchr((const char *)g_sim_rx_buf, '>')) return 0;

    Delay_ms(200);

    /* 5. Truyen noi dung tin nhan va ket thuc bang Ctrl+Z (0x1A) */
    SIM4G_ClearRx();
    SIM4G_SendStr(message);
    Delay_ms(100);
    SIM4G_SendByte(ctrl_z);

    /* 6. Cho phan hoi +CMGS va OK tu nha mang (toi da 25 giay) */
    t_start = GetTick_ms();
    while ((GetTick_ms() - t_start) < 25000)
    {
        if (u_strstr((const char *)g_sim_rx_buf, "+CMGS") || u_strstr((const char *)g_sim_rx_buf, "OK"))
        {
            g_sim_sms_sent = 1;
            return 1;
        }
        if (u_strstr((const char *)g_sim_rx_buf, "ERROR"))
        {
            return 0;
        }
        Delay_ms(50);
    }

    return 0;
}

uint8_t SIM4G_SendSMS(const char *phone_number, const char *message)
{
    uint8_t ok = SIM4G_SendSMS_Internal(phone_number, message);
    if (!ok && phone_number[0] == '0')
    {
        /* Thu lai voi ma quoc te +84 neu can ma vung (+84792394699) */
        char intl_num[32];
        intl_num[0] = '+'; intl_num[1] = '8'; intl_num[2] = '4';
        uint32_t i = 3;
        for (uint32_t j = 1; phone_number[j] && i < 30; j++) {
            intl_num[i++] = phone_number[j];
        }
        intl_num[i] = '\0';
        Delay_ms(1000);
        ok = SIM4G_SendSMS_Internal(intl_num, message);
    }
    return ok;
}
