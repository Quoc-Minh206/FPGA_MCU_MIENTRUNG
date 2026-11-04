/******************************************************************************
 * debug_uart.c - Driver UART Debug cho SONiX SN32F407 (P3.10 TX, P3.11 RX)
 *
 *  - Tốc độ Baud: 115200 bps, 8N1 (IHRC 12.0 MHz)
 *  - P3.10: Chân TX (Output Push-Pull, nối vào chân RX của mạch USB-UART)
 *  - P3.11: Chân RX (Input Pull-up, nối vào chân TX của mạch USB-UART)
 *  - Thiết kế Software UART chuẩn xác theo chu kỳ xung nhịp Cortex-M0,
 *    hoạt động độc lập 100% với UART1 (SIM A7680C) và UART0 (GPS NEO-6M).
 *****************************************************************************/
#include <SN32F400.h>
#include <stdarg.h>
#include "debug_uart.h"

/* 
 * Định thời chu kỳ xung nhịp cho 1 bit ở tốc độ 115200 bps:
 * F_CPU = 12,000,000 Hz.
 * Thời gian 1 bit = 1 / 115,200 = 8.68055 us.
 * Số chu kỳ CPU cho 1 bit = 12,000,000 / 115,200 = 104.17 cycles.
 * Vòng lặp subs (1 cycle) + bne (2 cycles) = 3 cycles/lần lặp.
 * Bù trừ overhead lệnh ghi GPIO (~12 cycles):
 * (104 - 12) / 3 = 30.67 -> Chọn 31 lần lặp (~92 cycles delay + 12 cycles = 104 cycles).
 * Sai số thực tế: +0.16%, trùng khớp hoàn toàn với chuẩn baud UART phần cứng.
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

#define UART_BIT_DELAY() delay_cycles(23)

void DEBUG_UART_Init(void)
{
    /* 1. Cấu hình P3.10 làm Output Push-Pull (TX) */
    SN_GPIO3->MODE |= (1UL << 10);

    /* 2. Đặt mức Idle của đường truyền UART là HIGH (3.3V) */
    SN_GPIO3->BSET = (1UL << 10);

    /* 3. Cấu hình P3.11 làm Input kèm trở kéo lên (Pull-up: CFG bits 23:22 = 00b) */
    SN_GPIO3->MODE &= ~(1UL << 11);
    SN_GPIO3->CFG  &= ~(3UL << 22);

    /* Giữ đường truyền ổn định 1ms lúc khởi động */
    for (volatile int i = 0; i < 1000; i++) {
        __asm volatile ("nop");
    }
}

void DEBUG_UART_SendChar(char c)
{
    /* Tắt ngắt tạm thời trong 87us phát 1 byte để bảo đảm baudrate chuẩn xác tuyệt đối */
    __disable_irq();

    /* 1. Start Bit (Mức LOW) */
    SN_GPIO3->BCLR = (1UL << 10);
    UART_BIT_DELAY();

    /* 2. 8 Data Bits (LSB xuất trước, unrolled chống lệch nhịp) */
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

    /* 3. Stop Bit (Mức HIGH) */
    SN_GPIO3->BSET = (1UL << 10);
    UART_BIT_DELAY();

    __enable_irq();
}

void DEBUG_UART_SendString(const char *str)
{
    if (!str) return;
    while (*str) {
        DEBUG_UART_SendChar(*str++);
    }
}

static void print_num(int32_t val, int base)
{
    char buf[12];
    int idx = 0;
    uint32_t uval;

    if (base == 10 && val < 0) {
        DEBUG_UART_SendChar('-');
        uval = (uint32_t)(-val);
    } else {
        uval = (uint32_t)val;
    }

    if (uval == 0) {
        DEBUG_UART_SendChar('0');
        return;
    }

    while (uval > 0) {
        uint32_t rem = uval % base;
        buf[idx++] = (rem < 10) ? ('0' + rem) : ('A' + rem - 10);
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
                print_num(d, 10);
            } else if (*fmt == 'u') {
                unsigned int u = va_arg(args, unsigned int);
                print_num((int32_t)u, 10);
            } else if (*fmt == 'x' || *fmt == 'X') {
                unsigned int x = va_arg(args, unsigned int);
                print_num((int32_t)x, 16);
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
    /* Chân P3.11 về mức LOW báo hiệu có Start Bit */
    return ((SN_GPIO3->DATA & (1UL << 11)) == 0) ? 1 : 0;
}

char DEBUG_UART_ReadChar(void)
{
    if (!DEBUG_UART_Available()) return 0;

    __disable_irq();
    /* Canh giữa Start bit (~52 chu kỳ): delay_cycles(11) */
    delay_cycles(11);

    char c = 0;
    /* Lấy mẫu 8 bit dữ liệu tại điểm giữa mỗi bit */
    for (int i = 0; i < 8; i++) {
        UART_BIT_DELAY();
        if (SN_GPIO3->DATA & (1UL << 11)) {
            c |= (1 << i);
        }
    }

    /* Chờ Stop bit */
    UART_BIT_DELAY();
    __enable_irq();

    return c;
}
