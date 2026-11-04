/******************************************************************************
 * @file    sim_driver.c
 * @brief   Triển khai bộ thư viện giao tiếp Module SIM qua UART (Gửi tin nhắn SMS)
 * @author  Embedded Firmware Engineer
 *
 * THÔNG SỐ PHẦN CỨNG & SDK MỤC TIÊU:
 * - Vi điều khiển (MCU): SONiX SN32F407 (ARM Cortex-M0 @ 12MHz IHRC)
 * - Thư viện/Framework : CMSIS & Bare-metal Registers (SN32F400.h)
 * - Cổng UART sử dụng  : UART1 (P2.2 UTXD1, P2.3 URXD1 @ 115200 bps 8N1)
 ******************************************************************************/
#include "sim_driver.h"
#include <SN32F400.h>
#include <stdio.h>
#include <string.h>

/*============================================================================*/
/* ĐỊNH NGHĨA THANH GHI BARE-METAL & CẤU HÌNH BỘ ĐỆM                          */
/*============================================================================*/
#define UART1_CLKEN_BIT     (1UL << 17)     /* SYS1_AHBCLKEN: Clock UART1    */
#define UART1_RST_BIT       (1UL << 17)     /* SYS1_PRST: Reset UART1        */
#define PFPA_UART1_P22_P23  ((1UL << 2) | (1UL << 0)) /* Ánh xạ chân P2.2 & P2.3*/

#define UART_LS_RDR         (1UL << 0)      /* Cờ có dữ liệu trong RX FIFO   */
#define UART_LS_THRE        (1UL << 5)      /* Cờ thanh ghi giữ phát trống   */
#define UART_LS_TEMT        (1UL << 6)      /* Cờ phát xong hoàn toàn        */

#define SIM_RX_BUF_SIZE     256             /* Kích thước bộ đệm nhận UART   */

/* Bộ đệm nhận UART toàn cục nội bộ */
static volatile char     s_rx_buffer[SIM_RX_BUF_SIZE];
static volatile uint16_t s_rx_index = 0;

/* Biến đếm thời gian hệ thống từ SysTick */
extern volatile uint32_t g_tick_ms;

/*============================================================================*/
/* 2. TÁCH LỚP PHẦN CỨNG (HARDWARE ABSTRACTION LAYER - HAL NỘI BỘ)            */
/*============================================================================*/

/**
 * @brief  Hàm phát 1 byte dữ liệu qua UART1
 */
static void MCU_UART_SendByte(uint8_t byte)
{
    uint32_t timeout = 100000UL;
    /* Chờ thanh ghi phát đệm trống (THRE) */
    while (((SN_UART1->LS & UART_LS_THRE) == 0UL) && (--timeout > 0UL));
    SN_UART1->TH = (uint32_t)byte;
}

/**
 * @brief  Hàm phát chuỗi ký tự qua UART1
 */
static void MCU_UART_SendString(const char *str)
{
    if (!str) return;
    while (*str) {
        MCU_UART_SendByte((uint8_t)*str++);
    }
    /* Chờ đẩy hoàn toàn byte cuối cùng ra ngoài đường truyền (TEMT) */
    uint32_t timeout = 100000UL;
    while (((SN_UART1->LS & UART_LS_TEMT) == 0UL) && (--timeout > 0UL));
}

/**
 * @brief  Hàm tạo trễ theo mili-giây (Dựa trên SysTick counter g_tick_ms)
 */
static void MCU_Delay_ms(uint32_t ms)
{
    uint32_t start = g_tick_ms;
    while ((g_tick_ms - start) < ms) {
        __asm volatile ("nop");
    }
}

/**
 * @brief  Hàm xóa sạch bộ đệm nhận phần mềm và xả sạch FIFO phần cứng
 */
static void SIM_Clear_Rx_Buffer(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Xả sạch toàn bộ byte còn tồn đọng trong RX FIFO của UART phần cứng */
    volatile uint32_t dummy;
    while (SN_UART1->LS & UART_LS_RDR) {
        dummy = SN_UART1->RB;
    }
    (void)dummy;

    /* Xóa sạch bộ đệm mảng RAM */
    s_rx_index = 0;
    memset((void *)s_rx_buffer, 0, sizeof(s_rx_buffer));

    if (!primask) {
        __enable_irq();
    }
}

/**
 * @brief  Hàm chờ phản hồi chuỗi mong muốn từ module SIM trong timeout_ms
 * @param  expected_response: Chuỗi ký tự mong muốn (ví dụ: "OK", ">", "+CMGS:")
 * @param  timeout_ms:       Thời gian chờ tối đa (ms)
 * @return SIM_OK nếu tìm thấy chuỗi, SIM_ERROR nếu gặp "ERROR", SIM_TIMEOUT nếu hết giờ
 */
static SIM_Status_t SIM_WaitForResponse(const char *expected_response, uint32_t timeout_ms)
{
    uint32_t start_time = g_tick_ms;

    while ((g_tick_ms - start_time) < timeout_ms)
    {
        /* Kiểm tra chuỗi mong muốn bằng strstr */
        if (expected_response && strstr((const char *)s_rx_buffer, expected_response) != NULL) {
            return SIM_OK;
        }

        /* Kiểm tra lỗi từ modem */
        if (strstr((const char *)s_rx_buffer, "ERROR") != NULL ||
            strstr((const char *)s_rx_buffer, "+CMS ERROR") != NULL) {
            return SIM_ERROR;
        }

        MCU_Delay_ms(2);
    }

    return SIM_TIMEOUT;
}

/*============================================================================*/
/* 3. TRÌNH PHỤC VỤ NGẮT UART1 (UART RX ISR)                                   */
/*============================================================================*/
/**
 * @brief  Ngắt nhận dữ liệu UART1: Lưu trực tiếp byte nhận được vào s_rx_buffer
 */
void UART1_IRQHandler(void)
{
    uint32_t ii = SN_UART1->II;

    /* Kiểm tra cờ có ngắt xảy ra (Bit 0 của II = 0) */
    while ((ii & 0x01UL) == 0UL)
    {
        uint32_t int_id = (ii >> 1) & 0x07UL;

        if (int_id == 3UL) {
            /* Line Status error -> Đọc LS để xóa */
            volatile uint32_t ls = SN_UART1->LS;
            (void)ls;
        }
        else if (int_id == 2UL || int_id == 6UL) {
            /* RDA (Receive Data Available) hoặc CTO (Character Timeout) */
            while (SN_UART1->LS & UART_LS_RDR) {
                uint8_t byte = (uint8_t)(SN_UART1->RB & 0xFFUL);
                if (s_rx_index < (SIM_RX_BUF_SIZE - 1U)) {
                    s_rx_buffer[s_rx_index++] = (char)byte;
                    s_rx_buffer[s_rx_index] = '\0'; /* Luôn kết thúc bằng null-terminator */
                }
            }
        }
        else {
            /* Ngắt khác: đọc xả để đảm bảo không kẹt */
            (void)SN_UART1->LS;
            (void)SN_UART1->RB;
            break;
        }
        ii = SN_UART1->II;
    }
}

/**
 * @brief  Khởi tạo cấu hình ngoại vi phần cứng UART1 (115200 bps 8N1)
 */
static void Hardware_UART1_Init(void)
{
    /* 1. Cấu hình chân GPIO P2.2 làm UTXD1 (Output), P2.3 làm URXD1 (Input Pull-up) */
    SN_GPIO2->BSET |= (1UL << 2);
    SN_GPIO2->MODE  = (SN_GPIO2->MODE | (1UL << 2)) & ~(1UL << 3);
    SN_GPIO2->CFG  &= ~((3UL << 4) | (3UL << 6));

    /* 2. Ánh xạ chân ngoại vi PFPA_UART1 */
    SN_PFPA->UART1 = (SN_PFPA->UART1 & ~0x0FUL) | PFPA_UART1_P22_P23;

    /* 3. Bật xung nhịp AHB và Reset phần cứng UART1 */
    SN_SYS1->AHBCLKEN |= UART1_CLKEN_BIT;
    SN_SYS1->PRST     |= UART1_RST_BIT;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* 4. Cài đặt Baudrate 115200 bps @ 12MHz IHRC */
    SN_UART1->LC  = 0x83UL;
    SN_UART1->FD  = (1UL << 7) | (11UL << 4) | (7UL << 0); /* FD = 0x75 */
    SN_UART1->DLM = 0UL;
    SN_UART1->DLL = 4UL;
    SN_UART1->LC  = 0x03UL; /* Khóa DLAB */

    /* 5. Cấu hình FIFO và ngắt nhận */
    SN_UART1->FIFOCTRL = 0x07UL; /* Bật và Reset TX/RX FIFO */
    SN_UART1->IE       = 0x01UL; /* Bật ngắt nhận dữ liệu RDA */

    /* 6. Bật ngắt UART1 trong NVIC */
    NVIC_ClearPendingIRQ(UART1_IRQn);
    NVIC_SetPriority(UART1_IRQn, 2);
    NVIC_EnableIRQ(UART1_IRQn);

    /* 7. Kích hoạt ngoại vi UART1 (UARTEN | RXEN | TXEN) */
    SN_UART1->CTRL = 0xC1UL;
}

/*============================================================================*/
/* 4. HÀM KHỞI TẠO MODULE SIM (SIM_Init)                                       */
/*============================================================================*/
SIM_Status_t SIM_Init(void)
{
    SIM_Status_t status;

    /* Khởi tạo phần cứng UART1 nếu chưa bật */
    Hardware_UART1_Init();
    MCU_Delay_ms(200);

    /* BƯỚC 1: Gửi 1 byte mã ESC (0x1B) và delay 200ms để ép module thoát khỏi
     *         chế độ dấu nhắc '>' nếu trước đó reset giữa chừng hoặc bị kẹt */
    MCU_UART_SendByte(0x1B);
    MCU_Delay_ms(200);

    /* BƯỚC 2: Xóa sạch bộ đệm nhận */
    SIM_Clear_Rx_Buffer();

    /* BƯỚC 3: Gửi "AT\r\n", chờ "OK" (timeout 1000ms), thử lại tối đa 2 lần */
    uint8_t retry = 0;
    status = SIM_TIMEOUT;
    while (retry < 2)
    {
        SIM_Clear_Rx_Buffer();
        MCU_UART_SendString("AT\r\n");
        status = SIM_WaitForResponse("OK", 1000);
        if (status == SIM_OK) {
            break;
        }
        retry++;
        MCU_Delay_ms(200);
    }

    if (status != SIM_OK) {
        return status; /* Không nhận được phản hồi AT sau số lần thử */
    }

    /* Tắt tính năng lặp lại lệnh Echo để dễ phân tích chuỗi */
    SIM_Clear_Rx_Buffer();
    MCU_UART_SendString("ATE0\r\n");
    SIM_WaitForResponse("OK", 500);

    /* BƯỚC 4: Gửi "AT+CMGF=1\r\n" để vào chế độ Text Mode, chờ "OK" (timeout 1000ms) */
    SIM_Clear_Rx_Buffer();
    MCU_UART_SendString("AT+CMGF=1\r\n");
    status = SIM_WaitForResponse("OK", 1000);

    return status;
}

/*============================================================================*/
/* 5. HÀM GỬI TIN NHẮN SMS (SIM_SendSMS)                                      */
/*============================================================================*/
SIM_Status_t SIM_SendSMS(const char *phone_number, const char *message)
{
    char cmd_buffer[64];
    SIM_Status_t status;

    if (!phone_number || !message) {
        return SIM_ERROR;
    }

    /* BƯỚC 1: Xóa buffer nhận, định dạng lệnh bằng snprintf: "AT+CMGS=\"%s\"\r\n" và gửi đi */
    SIM_Clear_Rx_Buffer();
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+CMGS=\"%s\"\r\n", phone_number);
    MCU_UART_SendString(cmd_buffer);

    /* BƯỚC 2: Chờ phản hồi dấu nhắc ">" (timeout 3000ms).
     *         Nếu timeout, bắt buộc gửi byte 0x1B (ESC) để hủy phiên rồi return SIM_TIMEOUT */
    status = SIM_WaitForResponse(">", 3000);
    if (status != SIM_OK) {
        MCU_UART_SendByte(0x1B); /* Hủy phiên gửi dở dang */
        MCU_Delay_ms(150);
        SIM_Clear_Rx_Buffer();
        return SIM_TIMEOUT;
    }

    /* BƯỚC 3: Gửi chuỗi nội dung message (không kèm \r\n) */
    SIM_Clear_Rx_Buffer();
    MCU_UART_SendString(message);
    MCU_Delay_ms(50);

    /* BƯỚC 4: Gửi 1 byte duy nhất mã Ctrl+Z (0x1A) để phát lệnh gửi bản tin */
    MCU_UART_SendByte(0x1A);

    /* BƯỚC 5: Chờ phản hồi "OK" hoặc "+CMGS:" với timeout dài (15000ms).
     *         Nếu timeout hoặc gặp lỗi, gửi byte 0x1B (ESC) để giải phóng trạng thái trước khi return */
    uint32_t start_time = g_tick_ms;
    while ((g_tick_ms - start_time) < 15000UL)
    {
        /* Kiểm tra xác nhận thành công từ nhà mạng */
        if (strstr((const char *)s_rx_buffer, "+CMGS:") != NULL ||
            strstr((const char *)s_rx_buffer, "OK") != NULL) {
            SIM_Clear_Rx_Buffer();
            return SIM_OK;
        }

        /* Kiểm tra báo lỗi từ mạng viễn thông */
        if (strstr((const char *)s_rx_buffer, "ERROR") != NULL ||
            strstr((const char *)s_rx_buffer, "+CMS ERROR") != NULL) {
            MCU_UART_SendByte(0x1B); /* Giải phóng trạng thái */
            MCU_Delay_ms(100);
            SIM_Clear_Rx_Buffer();
            return SIM_ERROR;
        }

        MCU_Delay_ms(10);
    }

    /* Hết thời gian chờ 15s mà không có phản hồi ACK */
    MCU_UART_SendByte(0x1B); /* Giải phóng trạng thái modem */
    MCU_Delay_ms(100);
    SIM_Clear_Rx_Buffer();
    return SIM_TIMEOUT;
}
