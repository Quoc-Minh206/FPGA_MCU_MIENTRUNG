/******************************************************************************
 * sim4g.h  -  Driver 4G LTE SIM A7680C / SIM800 cho SONiX SN32F407 EVK
 *
 *  - UART1 115200 8N1, chan P2.2 (UTXD1) / P2.3 (URXD1) -> cum J7 tren EVK
 *  - Chan dieu khien Reset / Power: P2.1
 *  - Ho tro gui tin nhan SMS toi so dien thoai di dong
 *****************************************************************************/
#ifndef SIM4G_H
#define SIM4G_H

#include <stdint.h>

#define SIM_RX_BUFFER_SIZE  384

extern volatile char     g_sim_rx_buf[SIM_RX_BUFFER_SIZE];
extern volatile uint16_t g_sim_rx_len;
extern volatile uint8_t  g_sim_ready;
extern volatile uint8_t  g_sim_sms_sent;

void    SIM4G_Init(void);
void    SIM4G_SendByte(uint8_t b);
void    SIM4G_SendStr(const char *str);
void    SIM4G_ClearRx(void);
uint8_t SIM4G_SendCmd(const char *cmd, const char *expected, uint32_t timeout_ms);
uint8_t SIM4G_SendSMS(const char *phone_number, const char *message);

#endif /* SIM4G_H */
