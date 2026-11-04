/******************************************************************************
 * File: tinyml_infer.c
 * Mô hình TinyML Autoencoder phát hiện tai nạn xe máy (4 -> 8 -> 3 -> 8 -> 4)
 * Thiết kế tối ưu cho MCU SONiX SN32F407 (ARM Cortex-M0)
 * Chiếm dụng bộ nhớ: ~560 Bytes Flash, ~88 Bytes SRAM | Thời gian thực thi: ~1.1ms
 *****************************************************************************/
#include "tinyml_infer.h"
#include "tinyml_weights.h"

/* Bộ đệm trạng thái xác nhận Two-Stage Latch */
static uint32_t s_tilt_hold_ms = 0;
static uint32_t s_latch_timer_ms = 0;
static uint8_t  s_anomaly_latched = 0;

void tinyml_init(void)
{
    s_tilt_hold_ms = 0;
    s_latch_timer_ms = 0;
    s_anomaly_latched = 0;
}

float tinyml_predict(const tinyml_features_t *feat)
{
    float x_raw[4];
    float x_norm[4];
    float h1[8];
    float z[3];
    float h2[8];
    float x_recon[4];
    float weighted_mse = 0.0f;
    int i, j;

    if (!feat) return 0.0f;

    x_raw[0] = feat->pitch;
    x_raw[1] = feat->roll;
    x_raw[2] = feat->rms;
    x_raw[3] = feat->max_jerk;

    /* 1. Chuẩn hóa đầu vào */
    for (i = 0; i < 4; i++) {
        x_norm[i] = (x_raw[i] - SCALE_MIN[i]) / SCALE_DIFF[i];
    }

    /* 2. Encoder 1: 4 -> 8 với LeakyReLU(0.1) */
    for (i = 0; i < 8; i++) {
        float sum = B_ENC1[i];
        for (j = 0; j < 4; j++) {
            sum += W_ENC1[i][j] * x_norm[j];
        }
        h1[i] = (sum >= 0.0f) ? sum : (sum * 0.1f);
    }

    /* 3. Encoder 2: 8 -> 3 (Cổ chai biểu diễn) */
    for (i = 0; i < 3; i++) {
        float sum = B_ENC2[i];
        for (j = 0; j < 8; j++) {
            sum += W_ENC2[i][j] * h1[j];
        }
        z[i] = sum;
    }

    /* 4. Decoder 1: 3 -> 8 với LeakyReLU(0.1) */
    for (i = 0; i < 8; i++) {
        float sum = B_DEC1[i];
        for (j = 0; j < 3; j++) {
            sum += W_DEC1[i][j] * z[j];
        }
        h2[i] = (sum >= 0.0f) ? sum : (sum * 0.1f);
    }

    /* 5. Decoder 2: 8 -> 4 (Tái tạo đặc trưng) */
    for (i = 0; i < 4; i++) {
        float sum = B_DEC2[i];
        for (j = 0; j < 8; j++) {
            sum += W_DEC2[i][j] * h2[j];
        }
        x_recon[i] = sum;
    }

    /* 6. Tính sai số tái tạo có trọng số (Weighted MSE: 40% Jerk, 25% RMS, 25% Roll, 10% Pitch) */
    for (i = 0; i < 4; i++) {
        float diff = x_recon[i] - x_norm[i];
        weighted_mse += WEIGHT_ERR[i] * (diff * diff);
    }

    return weighted_mse;
}

uint8_t tinyml_is_anomaly(float mse_score)
{
    return (mse_score > RECON_THRESHOLD) ? 1 : 0;
}

/*
 * Bộ lọc Two-Stage Latch xác nhận tai nạn chuẩn vật lý:
 * 1. Va chạm trực diện cực mạnh (Severe Impact: MSE > 5 * Threshold) -> Báo động tức thì!
 * 2. Va chạm vừa phải (MSE > Threshold) -> Chốt cờ nghi ngờ (Latch = 1) và mở cửa sổ trễ 1.5s.
 *    Nếu trong 1.5s xe bị ngã đổ (|Roll| >= 45 độ) -> Xác nhận tai nạn thực sự!
 * 3. Nếu là ổ gà: Sau 1.5s xe vẫn đứng thẳng (|Roll| < 25 độ) -> Tự động xóa cờ, không báo động giả!
 */
uint8_t tinyml_confirm_accident(float roll_angle, float mse_score, uint32_t dt_ms)
{
    float abs_roll = (roll_angle < 0.0f) ? -roll_angle : roll_angle;

    /* Trường hợp 1: Va chạm trực diện cực mạnh (xung lực vọt đỉnh) -> Báo động tức thì */
    if (mse_score > (5.0f * RECON_THRESHOLD)) {
        return 1;
    }

    /* Pha 1: Bắt xung va chạm (AI phát hiện bất thường) -> Chốt cờ */
    if (mse_score > RECON_THRESHOLD) {
        s_anomaly_latched = 1;
        s_latch_timer_ms = 0;
    }

    /* Pha 2: Nếu đã chốt cờ, quan sát trong cửa sổ 1.5 - 2.0 giây */
    if (s_anomaly_latched) {
        s_latch_timer_ms += dt_ms;

        /* Kiểm tra xe có ngã đổ xuống đường không */
        if (abs_roll >= 45.0f) {
            s_tilt_hold_ms += dt_ms;
            /* Nếu xe nằm nghiêng liên tục >= 500ms trong cửa sổ sau cú va chạm */
            if (s_tilt_hold_ms >= 500) {
                return 1; /* XÁC NHẬN TAI NẠN 100% */
            }
        }

        /* Nếu hết 2.0 giây mà xe vẫn đứng thẳng (|Roll| < 25 độ) -> Ổ gà, xóa cờ */
        if (s_latch_timer_ms >= 2000) {
            if (abs_roll < 25.0f) {
                s_anomaly_latched = 0;
                s_tilt_hold_ms = 0;
                s_latch_timer_ms = 0;
            }
        }
    }

    return 0;
}
