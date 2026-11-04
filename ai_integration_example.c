/******************************************************************************
 * File: ai_integration_example.c
 * Dự án: Hộp Đen Thông Minh Phát Hiện Tai Nạn Xe Máy (Edge AI)
 * Mô tả: File code C mẫu minh họa cách tích hợp bộ suy luận TinyML Autoencoder
 *        vào vòng lặp chính (main loop) của vi điều khiển (MCU ARM Cortex-M0).
 *****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include "tinyml_infer.h"

/* Mô phỏng các hàm phần cứng ngoại vi */
void Hardware_Init(void) {
    /* Khởi tạo UART, I2C, SPI, SysTick 1ms... */
    printf("[System] Hardware peripherals initialized.\n");
}

void Sensor_Read_Dual_Path(float *pitch, float *roll, float *rms, float *jerk) {
    /* 
     * Dữ liệu nhận từ cảm biến MPU-6050 & Tiền xử lý DSP FIR 15-Tap:
     * - Nhánh 1 (FIR Low-pass 5Hz): Ước lượng Pitch, Roll
     * - Nhánh 2 (Raw / Unfiltered): Giữ thành phần động RMS, Jerk
     */
    *pitch = 12.5f;   /* Ví dụ: Xe đang ôm cua nhẹ */
    *roll  = 18.2f;
    *rms   = 1.02f;   /* Trọng lực ~1.0g bình thường */
    *jerk  = 2.45f;   /* Rung động mặt đường nhẹ */
}

void Trigger_Emergency_SOS(float roll_angle, float mse_val) {
    printf("\n=======================================================\n");
    printf("[CANH BAO KHAN CAP] XAC NHAN TAI NAN VA CHAM XE MAY!\n");
    printf(" - Sai so tai tao MSE: %.4f (Vuot nguong 0.0907)\n", mse_val);
    printf(" - Goc nghieng xe Roll: %.1f do (> 60 do duy tri > 1.5s)\n", roll_angle);
    printf(" -> Bat coi bao dong Buzzer (PWM / GPIO)\n");
    printf(" -> Trich xuat kinh vi do GPS NEO-6M\n");
    printf(" -> Gui ban tin SOS kem vi tri qua song 4G LTE SIM A7680C\n");
    printf("=======================================================\n\n");
}

int main(void) {
    Hardware_Init();

    /* 1. Khởi tạo mô hình TinyML và bộ lọc trễ xác nhận đổ ngã */
    tinyml_init();
    printf("[TinyML] Autoencoder (4-8-3-8-4) initialized successfully.\n");
    printf("[TinyML] Reconstruction Threshold: RECON_THRESHOLD = %.4f\n", RECON_THRESHOLD);

    /* Giả lập vòng lặp định thời 50ms (tương ứng tần số suy luận 20Hz) */
    uint32_t current_time_ms = 0;
    const uint32_t loop_step_ms = 50;

    for (int step = 0; step < 40; step++) {
        current_time_ms += loop_step_ms;

        /* A. Thu thập 4 đặc trưng Dual-Path */
        tinyml_features_t feat;
        Sensor_Read_Dual_Path(&feat.pitch, &feat.roll, &feat.rms, &feat.max_jerk);

        /* Mô phỏng giả lập một vụ va chạm ở bước thứ 10 */
        if (step >= 10) {
            feat.pitch    = 4.5f;
            feat.roll     = 78.5f;   /* Xe ngã nghiêng 78.5 độ */
            feat.rms      = 3.85f;   /* Xung gia tốc lớn */
            feat.max_jerk = 24.5f;   /* Độ giật vọt đỉnh */
        }

        /* B. Thực thi suy luận AI Forward Pass (Chỉ mất 1.18ms trên Cortex-M0) */
        float mse_score = tinyml_predict(&feat);

        /* C. Kiểm tra trạng thái dị thường (Anomaly Detection) */
        uint8_t is_anomaly = tinyml_is_anomaly(mse_score);

        /* D. Bộ lọc trễ 1.5s xác nhận đổ ngã (Loại bỏ 100% báo giả khi sụp ổ gà) */
        uint8_t is_crash = tinyml_confirm_accident(feat.roll, mse_score, loop_step_ms);

        printf("[T=%4d ms] Pitch=%4.1f | Roll=%4.1f | RMS=%.2f | Jerk=%4.1f || MSE=%.4f [%s] -> %s\n",
               current_time_ms, feat.pitch, feat.roll, feat.rms, feat.max_jerk,
               mse_score,
               is_anomaly ? "ANOMALY" : "NORMAL",
               is_crash ? "CRASH CONFIRMED!" : "Safe");

        /* E. Kích hoạt cứu hộ khi tai nạn được xác nhận */
        if (is_crash) {
            Trigger_Emergency_SOS(feat.roll, mse_score);
            break;
        }
    }

    return 0;
}
