# Hệ Thống Hộp Đen Phát Hiện Tai Nạn Xe Máy & Định Vị Cứu Hộ Khẩn Cấp (Edge AI)
## Dự Án Vòng Chung Kết Cuộc Thi Thiết Kế FPGA & MCU Miền Trung 2026

Dự án phát triển hệ thống nhúng thông minh tích hợp **Gowin FPGA (Tang Nano 4K / OneKiwi 4K)** và **MCU SONiX SN32F407 (ARM Cortex-M0)** kết hợp mạng nơ-ron nhân tạo **TinyML Autoencoder** nhằm phát hiện sớm va chạm/tai nạn ngã xe máy theo thời gian thực, đồng thời tự động gửi tọa độ định vị GPS cứu hộ khẩn cấp qua mạng viễn thông **4G LTE (SIMCOM A7680C)**.

---

## 🚀 Tính Năng Nổi Bật

1. **Phần cứng kết hợp FPGA + MCU (Dual-Core Heterogeneous Architecture):**
   - **FPGA Gowin GW1NSR-4C (OneKiwi 4K):** Đảm nhiệm lấy mẫu dữ liệu cảm biến quán tính 6-trục **MPU6050** tốc độ cao, xử lý tiền xử lý tín hiệu DSP (bộ lọc FIR lọc thông thấp 5Hz tách trọng lực và trích xuất 4 đặc trưng Dual-Path: *Pitch, Roll, RMS, Jerk*). Phát xung ngắt cứng `WAKEUP_IRQ` (Pin 41) đánh thức MCU khi phát hiện chấn động.
   - **MCU SONiX SN32F407 (ARM Cortex-M0 @ 12MHz - 60MHz):** Đóng vai trò bộ xử lý trung tâm điều phối, chạy suy luận mô hình mạng nơ-ron **TinyML Autoencoder (4 -> 8 -> 3 -> 8 -> 4)**, định thời thuật toán xác nhận ngã xe Two-Stage Latch (chống báo giả khi sụp ổ gà).
2. **Hệ thống Định vị & Cứu hộ Thời Gian Thực:**
   - **u-blox NEO-6M GPS (UART0):** Trích xuất tọa độ kinh độ, vĩ độ NMEA ($GPRMC, $GPGGA) và tự động tạo đường dẫn trực tiếp Google Maps (`https://maps.google.com/?q=...`).
   - **SIMCOM A7680C 4G LTE (UART1):** Điều khiển tập lệnh AT Command gửi tin nhắn cảnh báo SOS kèm vị trí tai nạn đến số điện thoại khẩn cấp.
3. **Cơ chế Cảnh báo An Toàn HMI:**
   - Còi báo động Buzzer hoạt động theo nhịp điệu (150ms bíp / 250ms ngắt khi khẩn cấp; bíp chậm 200ms / 800ms khi đã gửi SMS thành công).
   - Nút bấm test thủ công **SW3** (P2.4) trên bo EVK Sonix và nút **K1/K2 (S1/S2)** trên bo FPGA.
   - Nút hủy báo động khẩn cấp **SW6** (P2.7) cho phép người lái xe tắt báo động trong trường hợp ngã nhẹ không nguy hiểm.

---

## 📁 Cấu Trúc Thư Mục Dự Án

```
main_project/
├── main.c                   # Mã nguồn chính điều khiển MCU SONiX SN32F407
├── system_shared.h          # Định nghĩa phần cứng, GPIO, thanh ghi Sonix & các macro
├── sim4g.c / sim4g.h        # Driver điều khiển module 4G LTE SIMCOM A7680C
├── gps.c / gps.h            # Driver giải mã chuỗi NMEA GPS u-blox NEO-6M
├── tinyml_infer.c / .h      # Bộ suy luận mạng nơ-ron TinyML Autoencoder & thuật toán Two-Stage Latch
├── tinyml_weights.h         # Trọng số và ngưỡng tái tạo của mô hình AI
├── MCU_MIENTRUNG.uvprojx    # Project Keil MDK-ARM uVision V5 cho MCU SONiX
├── FPGA_MIENTRUNG/          # Project Gowin EDA cho FPGA Gowin GW1NSR-4C
│   ├── src/fpga_trigger_top.v
│   └── impl/pnr/FPGA_MIENTRUNG.fs   # Bitstream nạp trực tiếp vào FPGA embFlash
├── constrain.pdf            # Sơ đồ gán chân (Pin Constraint) giữa FPGA, MCU và các ngoại vi
├── Schematic.drawio.png     # Sơ đồ nguyên lý kết nối toàn hệ thống
├── test_mpu/                # Dự án kiểm thử cảm biến MPU6050
├── test_sd/                 # Dự án kiểm thử thẻ nhớ MicroSD lưu log hộp đen
└── test_sim/                # Dự án kiểm thử độc lập module 4G LTE A7680C
```

---

## 🔌 Sơ Đồ Đấu Nối Phần Cứng (Pin Assignment)

### 1. Kết nối FPGA Gowin OneKiwi 4K ↔ MCU SONiX SN32F407
| Chức năng | Chân FPGA (Tang Nano 4K) | Chân MCU SONiX (SN32F407) | Vị trí Header trên EVK |
|---|---|---|---|
| **WAKEUP_IRQ** | Pin 41 (Dây Đỏ) | P2.9 | Cụm J7 - Chân 5 (P29) |
| **SCK** | Pin 44 (Dây Xanh biển) | P1.0 | Cụm J12 - Chân 2 (SCK_P10) |
| **MOSI** | Pin 42 (Dây Xanh lá) | P1.2 | Cụm J12 - Chân 4 (MOSI_P12) |
| **MISO** | Pin 43 (Dây Tím) | P1.1 | Cụm J12 - Chân 3 (MISO_P11) |
| **CS_N** | Pin 46 (Dây Trắng) | P1.8 | Cụm J12 - Chân 1 (SEL_P18) |

### 2. Ngoại Vi GPS NEO-6M & SIM 4G LTE A7680C ↔ MCU SONiX
| Ngoại vi | Chân ngoại vi | Chân MCU SONiX | Vị trí Header trên EVK |
|---|---|---|---|
| **GPS NEO-6M** | TXD (GPS phát) | P3.2 (URXD0) | Cụm J10 - Chân 2 (URX_P32) |
| **GPS NEO-6M** | RXD (GPS nhận) | P3.1 (UTXD0) | Cụm J10 - Chân 1 (UTX_P31) |
| **SIM A7680C** | TXD (SIM phát) | P2.3 (URXD1) | Cụm J7 - Chân 3 (P23) |
| **SIM A7680C** | RXD (SIM nhận) | P2.2 (UTXD1) | Cụm J7 - Chân 2 (P22) |
| **Buzzer Còi** | VCC / Signal | P2.5 | Cọc P25 (gần SW4) & PZ1 onboard (P3.0) |
| **LED Cảnh báo** | Anode (+) | P2.6 | Cọc P26 (gần SW5) & D7 onboard (P3.9) |
| **Nút Test Giả Lập** | Nút SW3 | P2.4 (Pull-up) | Phím SW3 onboard |
| **Nút Hủy SOS** | Nút SW6 | P2.7 (Pull-up) | Cọc P27 gần SW6 |

---

## 🛠 Hướng Dẫn Biên Dịch & Nạp Code

### 1. Nạp Code MCU SONiX SN32F407:
1. Cài đặt **Keil MDK-ARM V5** và gói thư viện **SONiX SN32F4 DFP**.
2. Mở file project `MCU_MIENTRUNG.uvprojx`.
3. Nhấn **F7** (Build Project) -> Đảm bảo `0 Error(s), 0 Warning(s)`.
4. Cắm cáp USB bo mạch Sonix SN-Link vào máy tính, nhấn **F8** (Download) để nạp vào ROM chip.

### 2. Nạp Bitstream FPGA Gowin OneKiwi 4K:
1. Mở phần mềm **Gowin Programmer** (hoặc `programmer_cli.exe`).
2. Chọn thiết bị: `GW1NSR-4C` (Device: `GW1NSR-LV4CQN48PC6/I5`).
3. Chọn file nạp: `FPGA_MIENTRUNG/impl/pnr/FPGA_MIENTRUNG.fs`.
4. Chọn chế độ nạp **embFlash** (hoặc SRAM) và tiến hành nạp.

---

## 👥 Nhóm Tác Giả & Bản Quyền
- **Cuộc thi:** Thiết kế FPGA & MCU Miền Trung 2026.
- **Repository:** [https://github.com/Quoc-Minh206/FPGA_MCU_MIENTRUNG](https://github.com/Quoc-Minh206/FPGA_MCU_MIENTRUNG)
