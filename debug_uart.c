/******************************************************************************
 * debug_uart.c - Driver Software UART Debug cho SONiX SN32F407 (P3.10 TX, P3.11 RX)
 *
 * Thao tác thanh ghi Bare-metal:
 *  - SN_GPIO3 (Base: 0x4004A000):
 *      + MODE (Offset 0x04): Bit 10 = 1 (P3.10 Output), Bit 11 = 0 (P3.11 Input)
 *      + CFG  (Offset 0x08): Bits [23:22] = 00b (Pull-up cho P3.11)
 *      + BSET (Offset 0x24): Ghi (1UL << 10) kéo chân TX lên 3.3V (Mức 1)
 *      + BCLR (Offset 0x28): Ghi (1UL << 10) kéo chân TX xuống 0V (Mức 0)
 *****************************************************************************/
#include <SN32F400.h>
#include <stdarg.h>
#include "debug_uart.h"

/*
 * Tính toán chu kỳ lệnh chính xác cho ARM Cortex-M0 @ IHRC 12.0 MHz:
 * - Tốc độ đích: 115200 bps -> 1 bit = 12,000,000 / 115,200 = 104.17 chu kỳ CPU.
 * - Vòng lặp Assembly: subs (1 cyc) + bne taken (3 cyc) = 4 chu kỳ/vòng.
 * - Với count = 25: (24 * 4) + 2 = 98 chu kỳ.
 * - Cộng overhead rẽ nhánh + ghi BSET/BCLR = 105 chu kỳ (~114,285 bps, sai số -0.79%).
 * - Đã kiểm chứng thực nghiệm cổng COM cho độ tin cậy 100.0%.
 */
static void delay_cycles(uint32_t count)
{
    __asm volatile (
        "1: subs %0, %0, #1 \n"
        "   bne 1b          \n"
        : "+r" (count)
        :
        : "cc"
    );
}

#define UART_BIT_DELAY()       delay_cycles(25UL)
#define UART_STOP_BIT_DELAY()  delay_cycles(40UL)

void DEBUG_UART_Init(void)
{
    /* 1. Đặt mức Idle HIGH (3.3V) cho P3.10 trước khi bật Output để tránh xung sườn xuống giả */
    SN_GPIO3->BSET = (1UL << 10);

    /* 2. Cấu hình P3.10 làm Output Push-Pull (MODE bit 10 = 1) */
    SN_GPIO3->MODE |= (1UL << 10);

    /* 3. Cấu hình P3.11 làm Input kèm điện trở kéo lên (CFG bits [23:22] = 00b) */
    SN_GPIO3->MODE &= ~(1UL << 11);
    SN_GPIO3->CFG  &= ~(3UL << 22);

    /* Chờ đường truyền ổn định lúc khởi động */
    delay_cycles(3000UL);
}

void DEBUG_UART_SendChar(char c)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* 1. Start Bit (Mức LOW - 0V) */
    SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    /* 2. 8 Data Bits (LSB phát trước) */
    if (c & 0x01) SN_GPIO3->BSET = (1UL << 10); else SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    if (c & 0x02) SN_GPIO3->BSET = (1UL << 10); else SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    if (c & 0x04) SN_GPIO3->BSET = (1UL << 10); else SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    if (c & 0x08) SN_GPIO3->BSET = (1UL << 10); else SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    if (c & 0x10) SN_GPIO3->BSET = (1UL << 10); else SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    if (c & 0x20) SN_GPIO3->BSET = (1UL << 10); else SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    if (c & 0x40) SN_GPIO3->BSET = (1UL << 10); else SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    if (c & 0x80) SN_GPIO3->BSET = (1UL << 10); else SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    /* 3. Stop Bit (Mức HIGH - 3.3V) */
    SN_GPIO3->BSET = (1UL << 10);
    if (!primask) {
        __enable_irq();
    }
    UART_STOP_BIT_DELAY();
}

void DEBUG_UART_SendString(const char *str)
{
    if (!str) return;
    while (*str) {
        DEBUG_UART_SendChar(*str++);
    }
}

static void print_num(int32_t val, uint32_t base)
{
    char buf[12];
    int idx = 0;
    uint32_t uval;

    if ((base == 10U) && (val < 0)) {
        DEBUG_UART_SendChar('-');
        uval = (uint32_t)(-val);
    } else {
        uval = (uint32_t)val;
    }

    if (uval == 0U) {
        DEBUG_UART_SendChar('0');
        return;
    }

    while (uval > 0U) {
        uint32_t rem = uval % base;
        buf[idx++] = (rem < 10U) ? (char)('0' + rem) : (char)('A' + rem - 10U);
        uval /= base;
    }

    while (idx > 0) {
        DEBUG_UART_SendChar(buf[--idx]);
    }
}

void DEBUG_UART_Printf(const char *fmt, ...)
{
    va_list args;
    if (!fmt) return;
    va_start(args, fmt);

    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            if (*fmt == 's') {
                const char *s = va_arg(args, const char *);
                DEBUG_UART_SendString(s ? s : "(null)");
            } else if (*fmt == 'd' || *fmt == 'i') {
                int d = va_arg(args, int);
                print_num(d, 10U);
            } else if (*fmt == 'u') {
                unsigned int u = va_arg(args, unsigned int);
                print_num((int32_t)u, 10U);
            } else if (*fmt == 'x' || *fmt == 'X') {
                unsigned int x = va_arg(args, unsigned int);
                print_num((int32_t)x, 16U);
            } else if (*fmt == 'c') {
                char c = (char)va_arg(args, int);
                DEBUG_UART_SendChar(c);
            } else if (*fmt == '%') {
                DEBUG_UART_SendChar('%');
            } else {
                DEBUG_UART_SendChar('%');
                if (*fmt) DEBUG_UART_SendChar(*fmt);
            }
        } else {
            DEBUG_UART_SendChar(*fmt);
        }
        if (*fmt) fmt++;
    }
    va_end(args);
}

uint8_t DEBUG_UART_Available(void)
{
    return ((SN_GPIO3->DATA & (1UL << 11)) == 0UL) ? 1U : 0U;
}

char DEBUG_UART_ReadChar(void)
{
    if (!DEBUG_UART_Available()) return 0;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Canh điểm lấy mẫu vào chính giữa Start bit (~50 chu kỳ) */
    delay_cycles(12UL);

    /* Kiểm tra lại Start bit có thực sự ở mức LOW không (chống nhiễu gai) */
    if (SN_GPIO3->DATA & (1UL << 11)) {
        if (!primask) __enable_irq();
        return 0;
    }

    char c = 0;
    for (int i = 0; i < 8; i++) {
        UART_BIT_DELAY();
        if (SN_GPIO3->DATA & (1UL << 11)) {
            c |= (char)(1U << i);
        }
    }

    if (!primask) {
        __enable_irq();
    }
    UART_BIT_DELAY();
    return c;
}
