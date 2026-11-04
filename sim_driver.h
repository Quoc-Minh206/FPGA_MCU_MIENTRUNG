/******************************************************************************
 * @file    sim_driver.h
 * @brief   Thư viện giao tiếp Module SIM qua UART (Gửi tin nhắn SMS)
 * @author  Embedded Firmware Engineer
 * @target  SONiX SN32F407 (ARM Cortex-M0) / Tương thích đa nền tảng
 ******************************************************************************/
#ifndef SIM_DRIVER_H
#define SIM_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* 1. ĐỊNH NGHĨA KIỂU DỮ LIỆU TRẢ VỀ                                          */
/*============================================================================*/
typedef enum {
    SIM_OK = 0,         /* Thành công hoàn toàn                               */
    SIM_TIMEOUT,        /* Hết thời gian chờ phản hồi từ module SIM           */
    SIM_ERROR           /* Module SIM trả về ERROR hoặc cấu hình thất bại     */
} SIM_Status_t;

/*============================================================================*/
/* 2. CÁC HÀM GIAO TIẾP NGOÀI (PUBLIC API)                                    */
/*============================================================================*/

/**
 * @brief  Khởi tạo ngoại vi phần cứng UART1 và module SIM
 *         Bao gồm cơ chế giải phóng thoát khỏi chế độ '>' nếu bị treo trước đó.
 * @return SIM_Status_t: SIM_OK nếu cấu hình thành công, ngược lại trả về mã lỗi.
 */
SIM_Status_t SIM_Init(void);

/**
 * @brief  Gửi tin nhắn văn bản SMS tới số điện thoại chỉ định
 * @param  phone_number: Số điện thoại người nhận (Ví dụ: "+84792394699" hoặc "0792394699")
 * @param  message:      Nội dung tin nhắn văn bản (chuẩn ASCII không dấu)
 * @return SIM_Status_t: SIM_OK nếu gửi thành công và nhận ACK từ mạng, ngược lại trả về mã lỗi.
 */
SIM_Status_t SIM_SendSMS(const char *phone_number, const char *message);

/**
 * @brief  Trình phục vụ ngắt nhận dữ liệu UART (Gọi trong Vector Table hoặc ISR)
 */
void UART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* SIM_DRIVER_H */
