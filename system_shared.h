#ifndef SYSTEM_SHARED_H
#define SYSTEM_SHARED_H

#include <stdint.h>
#include <SN32F400.h>

/*
=============================================================================
 * 1. REGISTERS & PERIPHERAL BASE ADDRESSES
 *    SONiX SN32F407 (ARM Cortex-M0, 32KB Flash, 8KB SRAM)
=============================================================================
*/
#define SYS0_ANBCTRL    (*(volatile uint32_t *)(0x40060000UL + 0x00UL))
#define SYS0_RSTST      (*(volatile uint32_t *)(0x40060000UL + 0x14UL))
#define SYS0_LVDCTRL    (*(volatile uint32_t *)(0x40060000UL + 0x18UL))
#define SYS1_AHBCLKEN   (*(volatile uint32_t *)(0x4005E000UL + 0x00UL))
#define SYS1_APBCP1     (*(volatile uint32_t *)(0x4005E000UL + 0x08UL))
#define PMU_CTRL        (*(volatile uint32_t *)(0x40032000UL + 0x40UL))

// PFPA Peripheral Function Pin Assignment (Base: 0x40042000)
#define PFPA_UART0      (*(volatile uint32_t *)(0x40042000UL + 0x20UL)) // UART0 (GPS: P3.1 TX, P3.2 RX)
#define PFPA_UART1      (*(volatile uint32_t *)(0x40042000UL + 0x24UL)) // UART1 (4G: P2.2 TX, P2.3 RX)
#define PFPA_SPI0       (*(volatile uint32_t *)(0x40042000UL + 0x50UL)) // SPI0 (FPGA: P1.0, P1.1, P1.2, P1.8)

// GPIO Port Registers
#define GPIO1_DATA      (*(volatile uint32_t *)(0x40046000UL + 0x00UL))
#define GPIO1_MODE      (*(volatile uint32_t *)(0x40046000UL + 0x04UL))
#define GPIO1_CFG       (*(volatile uint32_t *)(0x40046000UL + 0x08UL))
#define GPIO1_BSET      (*(volatile uint32_t *)(0x40046000UL + 0x24UL))
#define GPIO1_BCLR      (*(volatile uint32_t *)(0x40046000UL + 0x28UL))

#define GPIO2_DATA      (*(volatile uint32_t *)(0x40048000UL + 0x00UL))
#define GPIO2_MODE      (*(volatile uint32_t *)(0x40048000UL + 0x04UL))
#define GPIO2_CFG       (*(volatile uint32_t *)(0x40048000UL + 0x08UL))
#define GPIO2_IS        (*(volatile uint32_t *)(0x40048000UL + 0x0CUL))
#define GPIO2_IBS       (*(volatile uint32_t *)(0x40048000UL + 0x10UL))
#define GPIO2_IEV       (*(volatile uint32_t *)(0x40048000UL + 0x14UL))
#define GPIO2_IE        (*(volatile uint32_t *)(0x40048000UL + 0x18UL))
#define GPIO2_RIS       (*(volatile uint32_t *)(0x40048000UL + 0x1CUL))
#define GPIO2_IC        (*(volatile uint32_t *)(0x40048000UL + 0x20UL))
#define GPIO2_BSET      (*(volatile uint32_t *)(0x40048000UL + 0x24UL))
#define GPIO2_BCLR      (*(volatile uint32_t *)(0x40048000UL + 0x28UL))

#define GPIO3_DATA      (*(volatile uint32_t *)(0x4004A000UL + 0x00UL))
#define GPIO3_MODE      (*(volatile uint32_t *)(0x4004A000UL + 0x04UL))
#define GPIO3_BSET      (*(volatile uint32_t *)(0x4004A000UL + 0x24UL))
#define GPIO3_BCLR      (*(volatile uint32_t *)(0x4004A000UL + 0x28UL))

// Watchdog & SysTick
#define WDT_CFG         (*(volatile uint32_t *)(0x40010000UL + 0x00UL))
#define WDT_TC          (*(volatile uint32_t *)(0x40010000UL + 0x08UL))
#define WDT_FEED        (*(volatile uint32_t *)(0x40010000UL + 0x0CUL))

/*
=============================================================================
 * 2. CÒI BUZZER & ĐÈN LED CẢNH BÁO (THEO BẢNG 5 FILE CONSTRAIN.PDF & ONBOARD)
 *    - Bảng 5 constrain.pdf:
 *      + Còi Buzzer chủ động: Chân P2.5 (Cọc P25 gần SW4), Active High (3.3V)
 *      + Đèn LED cảnh báo đỏ: Chân P2.6 (Cọc P26 gần SW5), Active High (3.3V)
 *      + Nút bấm hủy khẩn cấp: Chân P2.7 (Cọc P27 gần SW6), Active Low (0V)
 *    - Dự phòng Onboard bo Sonix EVK:
 *      + Còi PZ1 onboard: P3.0
 *      + LED D6 (P3.8 - Heartbeat), LED D7 (P3.9 - Warning)
=============================================================================
*/
// Còi Buzzer: Dao động kích còi PZ1 onboard (P3.0)
extern volatile uint8_t g_buzzer_enabled;
#define BUZZER_ON()        do { g_buzzer_enabled = 1; } while(0)
#define BUZZER_OFF()       do { g_buzzer_enabled = 0; GPIO3_BCLR = (1UL << 0); } while(0)

// Đèn LED cảnh báo tai nạn: Bật D7 (P3.9 onboard - Active Low)
#define ALERT_LED_ON()     (GPIO3_BCLR = (1UL << 9))
#define ALERT_LED_OFF()    (GPIO3_BSET = (1UL << 9))

// Đèn LED D6 (P3.8 onboard) - Heartbeat hệ thống
#define LED_D6_ON()        (GPIO3_BCLR = (1UL << 8))
#define LED_D6_OFF()       (GPIO3_BSET = (1UL << 8))

// Macro tương thích ngược
#define LED_D7_ON()        ALERT_LED_ON()
#define LED_D7_OFF()       ALERT_LED_OFF()
#define LED_D8_SPI_ON()    ALERT_LED_ON()
#define LED_D8_SPI_OFF()   ALERT_LED_OFF()
#define LED_D9_AI_ON()     LED_D6_ON()
#define LED_D9_AI_OFF()    LED_D6_OFF()
#define LED_D10_GPS_ON()   // GPS status
#define LED_D10_GPS_OFF()  
#define LED_D11_4G_ON()    ALERT_LED_ON()
#define LED_D11_4G_OFF()   ALERT_LED_OFF()

/*
=============================================================================
 * 3. SHARED GLOBAL VARIABLES
=============================================================================
*/
extern volatile int16_t  g_features[4];
extern volatile uint8_t  g_spi_pkt_ready;
extern char              g_gps_coord[32];
extern char              g_gps_maps_link[64];

#endif /* SYSTEM_SHARED_H */
