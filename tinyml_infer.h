/******************************************************************************
 * File: tinyml_infer.h
 * Dự án: Hộp Đen Thông Minh Phát Hiện Tai Nạn Xe Máy (TinyML Edge AI)
 * Mục tiêu phần cứng: MCU SONiX SN32F407 (ARM Cortex-M0) & FPGA GOWIN
 *****************************************************************************/
#ifndef __TINYML_INFER_H__
#define __TINYML_INFER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Cấu trúc chứa 4 đặc trưng Dual-Path đầu vào của Autoencoder */
typedef struct {
    float pitch;      /* Góc chúi Pitch (độ) - từ Nhánh Static FIR 5Hz */
    float roll;       /* Góc nghiêng Roll (độ) - từ Nhánh Static FIR 5Hz */
    float rms;        /* Gia tốc Dynamic RMS (g) - đã khử DC 1.0g */
    float max_jerk;   /* Độ giật 3D Vector Jerk (g/s) */
} tinyml_features_t;

/**
 * @brief Khởi tạo mô hình TinyML và bộ đệm trạng thái
 */
void tinyml_init(void);

/**
 * @brief Thực thi suy luận Forward Pass Autoencoder 4 -> 8 -> 3 -> 8 -> 4
 * @param feat Con trỏ tới cấu trúc 4 đặc trưng
 * @return Sai số tái tạo có trọng số (Weighted MSE)
 */
float tinyml_predict(const tinyml_features_t *feat);

/**
 * @brief Kiểm tra xem sai số tái tạo có vượt ngưỡng dị thường không
 * @param mse_score Sai số tính từ tinyml_predict()
 * @return 1 nếu là dị thường, 0 nếu là bình thường
 */
uint8_t tinyml_is_anomaly(float mse_score);

/**
 * @brief Bộ lọc Two-Stage Latch xác nhận tai nạn chuẩn vật lý (chống báo giả ổ gà)
 * @param roll_angle Góc Roll hiện tại của xe (độ)
 * @param mse_score Sai số tái tạo hiện tại
 * @param dt_ms Thời gian bước nhảy (ví dụ: 50ms)
 * @return 1 nếu xác nhận TAI NẠN THỰC SỰ, 0 nếu an toàn/ổ gà
 */
uint8_t tinyml_confirm_accident(float roll_angle, float mse_score, uint32_t dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* __TINYML_INFER_H__ */
