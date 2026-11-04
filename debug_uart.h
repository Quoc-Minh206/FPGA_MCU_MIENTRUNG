/******************************************************************************
 * debug_uart.h - Driver UART Debug cho SONiX SN32F407 (P3.10 TX, P3.11 RX)
 *
 *  - Cấu hình: 115200 bps, 8-bit data, no parity, 1 stop bit (115200 8N1)
 *  - P3.10: Chân UART TX (nối với RX của USB-UART)
 *  - P3.11: Chân UART RX (nối với TX của USB-UART)
 *  - Hoạt động độc lập 100% với UART1 (A7680C 4G) và UART0 (GPS NEO-6M).
 *****************************************************************************/
#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>

void    DEBUG_UART_Init(void);
void    DEBUG_UART_SendChar(char c);
void    DEBUG_UART_SendString(const char *str);
void    DEBUG_UART_Printf(const char *fmt, ...);
uint8_t DEBUG_UART_Available(void);
char    DEBUG_UART_ReadChar(void);

#endif /* DEBUG_UART_H */
