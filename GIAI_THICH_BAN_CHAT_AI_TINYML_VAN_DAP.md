# TÀI LIỆU TOÀN DIỆN VỀ BẢN CHẤT AI TINYML & HỆ THỐNG PHÁT HIỆN TAI NẠN XE MÁY
> **Dành cho:** Chuẩn bị vấn đáp, bảo vệ đồ án tốt nghiệp / cuộc thi thiết kế vi mạch & hệ thống nhúng (FPGA & MCU).  
> **Nền tảng phần cứng:** Vi điều khiển SONiX SN32F407 (ARM Cortex-M0) + FPGA Gowin GW1NSR-4C (OneKiwi 4K / Tang Nano 4K).  
> **Ngôn ngữ & Công cụ:** Python (PyTorch, SciPy, NumPy) $\rightarrow$ C thuần nhúng bare-metal (Zero external library).

---

## MỤC LỤC CHI TIẾT
1. [Tổng quan Triết lý Thiết kế: Tại sao lại là TinyML Edge AI?](#1-tong-quan-triet-ly-thiet-ke-tai-sao-lai-la-tinyml-edge-ai)
2. [Bản chất Vật lý của Tai nạn Xe máy vs. Nhiễu môi trường](#2-ban-chat-vat-ly-cua-tai-nan-xe-may-vs-nhieu-moi-truong)
3. [Tầng Tiền xử lý DSP & Kiến trúc Lọc kép Dual-Path](#3-tang-tien-xu-ly-dsp--kien-truc-loc-kep-dual-path)
4. [Bản chất Toán học của Mô hình Autoencoder](#4-ban-chat-toan-hoc-cua-mo-hinh-autoencoder)
5. [Quy trình Huấn luyện & Đóng băng Trọng số (C Export)](#5-quy-trinh-huan-luyen--dong-bang-trong-so-c-export)
6. [Bộ lọc Xác nhận 2 Giai đoạn (Two-Stage Latch State Machine)](#6-bo-loc-xac-nhan-2-giai-doan-two-stage-latch-state-machine)
7. [Phân công Nhiệm vụ Phần cứng: FPGA vs. MCU Sonix](#7-phan-cong-nhiem-vu-phan-cung-fpga-vs-mcu-sonix)
8. [Bộ 15 Câu hỏi Vấn đáp "Tử huyệt" & Cách trả lời Đanh thép trước Hội đồng](#8-bo-15-cau-hoi-van-dap-tu-huyet--cach-tra-loi-danh-thep-truoc-hoi-dong)

---

## 1. TỔNG QUAN TRIẾT LÝ THIẾT KẾ: TẠI SAO LẠI LÀ TINYML EDGE AI?

### 1.1. Bài toán thực tế
Hệ thống phát hiện tai nạn xe máy tự động (eCall / SOS Box) có nhiệm vụ cứu sống nạn nhân bằng cách gửi tin nhắn SMS chứa tọa độ GPS chính xác tới người thân và trung tâm cấp cứu trong "Khung giờ vàng" (Golden Hour). Tuy nhiên, môi trường chuyển động của xe máy khắc nghiệt hơn ô tô gấp nhiều lần:
- Xe máy liên tục rung lắc từ động cơ đốt trong (30 - 45 Hz).
- Người lái thường xuyên ôm cua (xe nghiêng tới $35^\circ - 40^\circ$), vấp ổ gà, leo lề, gờ giảm tốc.

### 1.2. Tại sao các phương pháp truyền thống thất bại?
1. **Dùng ngưỡng gia tốc cố định (Rule-based: ví dụ $Acc > 2.5g$):**
   - Xe qua gờ giảm tốc hoặc sụp ổ gà sâu có thể tạo xung đỉnh lên tới $3.0g - 4.0g$. Hệ thống sẽ phát tín hiệu cấp cứu giả liên tục $\rightarrow$ Gây hoảng loạn và lãng phí tài nguyên cứu hộ.
2. **Dùng ngưỡng góc nghiêng cố định (ví dụ $|Roll| > 45^\circ$):**
   - Xe khi ôm cua gắt hoặc khi dừng dựng chân chống nghiêng có góc nghiêng rất lớn. Nếu chỉ dùng góc nghiêng, hệ thống sẽ báo tai nạn ngay khi người dùng vừa đỗ xe.
3. **Gửi dữ liệu lên Cloud để AI phân tích:**
   - Khi tai nạn xảy ra ở đèo dốc, vùng sâu vùng xa, sóng 4G chập chờn hoặc mất hẳn, hệ thống Cloud hoàn toàn bị tê liệt.
   - Độ trễ truyền tải Cloud (Latency) từ 1 - 3 giây là quá chậm để kích hoạt các phản ứng an toàn tức thời.

$\Rightarrow$ **Giải pháp bắt buộc:** **TinyML Edge AI (Trí tuệ nhân tạo biên siêu nhỏ gọn)**. Toàn bộ thuật toán AI phải chạy trực tiếp (On-device Inference) bên trong bộ nhớ của vi điều khiển Sonix SN32F407 (32KB Flash, 8KB RAM), xử lý cục bộ độc lập 100% không phụ thuộc Internet.

---

## 2. BẢN CHẤT VẬT LÝ CỦA TAI NẠN XE MÁY VS. NHIỄU MÔI TRƯỜNG

Mô hình AI của dự án không phải là một "hộp đen" đoán mò, mà được xây dựng trên **mô hình cơ học động lực học chuyển động (Crash Kinematics)**:

```
[Thời điểm t = 0]          [50ms - 200ms]                [200ms - 2.0s]              [Sau 2.0s]
  Xe chạy bình thường  --->  VA CHẠM CỰC MẠNH (Impact) --->  NGÃ ĐỔ XE (Resting)  --->  NẰM BẤT ĐỘNG
  - Gia tốc ~ 1.0g           - Xung lực vọt đỉnh            - Trục Ay hứng 1.0g         - Không chuyển động
  - Roll < 15 độ             - Jerk cực lớn (g/s)           - Roll > 45 - 80 độ         - Động cơ tắt
```

### So sánh bản chất vật lý giữa các hành vi:
| Đặc tính | Lái xe bình thường | Sụp ổ gà / Gờ giảm tốc | Ôm cua gắt | TAI NẠN THỰC SỰ |
| :--- | :--- | :--- | :--- | :--- |
| **Gia tốc động RMS** | Thấp ($0.05g - 0.2g$) | Cao đột biến ($1.5g - 3.5g$) | Trung bình ($0.2g - 0.4g$) | **Cực cao ($3.0g - 6.0g$)** |
| **3D Vector Jerk** | Rất thấp ($< 15 g/s$) | Trung bình ($30 - 60 g/s$) | Thấp ($< 10 g/s$) | **Cực đại ($> 100 - 300 g/s$)** |
| **Góc nghiêng Roll** | Thẳng đứng ($< 10^\circ$) | Không đổi ($< 15^\circ$) | Nghiêng tạm thời ($25^\circ - 38^\circ$) | **Đổ rạp mặt đường ($> 50^\circ - 85^\circ$)** |
| **Trạng thái sau 1.5s** | Bình thường | Xe tự đứng thẳng lại | Xe tự đứng thẳng lại | **Nằm nghiêng bất động liên tục** |

---

## 3. TẦNG TIỀN XỬ LÝ DSP & KIẾN TRÚC LỌC KÉP DUAL-PATH

Trước khi đưa vào mạng nơ-ron, tín hiệu thô từ cảm biến IMU MPU-6050 lấy mẫu ở tần số **$F_s = 100$ Hz** ($\Delta t = 10$ ms, đảm bảo định lý Nyquist vì động tác con người và xe máy nằm ở dải $< 20$ Hz).

```
Tín hiệu IMU thô (Ax, Ay, Az)
        │
        ├───> [Nhánh Tĩnh (Static Branch)] ───> Bộ lọc FIR 15-tap (Fc = 5Hz) ───> Tách trọng trường g (1.0g) ───> Tính góc Pitch & Roll
        │                                                    │
        └───> [Nhánh Động (Dynamic Branch)] <────────────────┘ (Trừ tín hiệu)
                     │
                     └───> Khử hoàn toàn 1.0g DC ───> Tính Dynamic RMS & 3D Vector Jerk
```

### 3.1. Nhánh Tĩnh (Static Path) - Bộ lọc thông thấp FIR 15-Tap ($F_c = 5$ Hz)
- **Mục đích:** Loại bỏ toàn bộ nhiễu rung từ động cơ xe máy (vốn dao động quanh dải $30 - 45$ Hz tại tua máy 1800 - 2700 RPM) và các xung xóc tức thời.
- **Kết quả:** Trích xuất vectơ trọng trường tĩnh $\vec{g} = [a_{xf}, a_{yf}, a_{zf}]$.
- **Công thức tính góc tuyệt đối (Euler Angles):**
  $$\text{Pitch} = \arctan\left(\frac{a_{xf}}{\sqrt{a_{yf}^2 + a_{zf}^2}}\right) \times \frac{180^\circ}{\pi}$$
  $$\text{Roll} = \arctan\left(\frac{a_{yf}}{a_{zf}}\right) \times \frac{180^\circ}{\pi}$$
- **Kỹ thuật đặc biệt (`lfilter_zi`):** Triệt tiêu hiện tượng quá độ (Transients) ở thời điểm khởi động, giúp bộ lọc đạt trạng thái xác lập ngay tại mẫu đầu tiên, không làm sai lệch góc nghiêng.

### 3.2. Nhánh Động (Dynamic Path) - Khử DC Trọng trường
- **Mục đích:** Khi xe di chuyển, cảm biến gia tốc luôn bị "ám" $1.0g$ gia tốc trọng trường của Trái Đất hướng xuống. Để đo chính xác lực va đập thuần túy do va chạm, ta lấy tín hiệu thô trừ đi nhánh tĩnh:
  $$\vec{a}_{dyn} = \vec{a}_{raw} - \vec{a}_{filt}$$
- **Đặc trưng 3: Dynamic RMS (Root Mean Square):**
  $$\text{RMS}_{dyn} = \sqrt{\frac{1}{N} \sum_{k=1}^{N} \left(a_{xd}^2[k] + a_{yd}^2[k] + a_{zd}^2[k]\right)}$$
  Đo tổng năng lượng rung động và xung kích trong cửa sổ trượt $W = 64$ mẫu ($0.64$ giây).
- **Đặc trưng 4: 3D Vector Jerk (Độ giật vi phân gia tốc):**
  $$\text{Jerk}[k] = \frac{\|\vec{a}_{dyn}[k] - \vec{a}_{dyn}[k-1]\|}{\Delta t}$$
  $$\text{Max Jerk} = \max_{k \in W} (\text{Jerk}[k]) \quad (\text{đơn vị: } g/s)$$
  *Bản chất vật lý:* Jerk đo tốc độ biến thiên của lực ($F = ma \Rightarrow \frac{dF}{dt} = m \cdot \text{Jerk}$). Khi xe đâm vào chướng ngại vật cứng, lực tăng vọt trong vài mili-giây, tạo ra Jerk khổng lồ ($> 150 g/s$), trong khi xe chạy bình thường hay qua ổ gà thì Jerk chỉ $< 30 g/s$.

---

## 4. BẢN CHẤT TOÁN HỌC CỦA MÔ HÌNH AUTOENCODER

### 4.1. Tại sao chọn Autoencoder (Unsupervised Anomaly Detection)?
Nếu dùng các mô hình phân loại thông thường (Binary Classifier: Logistic, SVM, Decision Tree, CNN):
- **Vấn đề "Mất cân bằng dữ liệu cực đoan" (Severe Class Imbalance):** Trong thực tế, dữ liệu lái xe an toàn chiếm $99.999\%$ tổng thời gian chạy xe. Dữ liệu va chạm thực tế rất hiếm và nguy hiểm để thu thập đủ mọi góc đâm. Nếu huấn luyện phân loại nhị phân thông thường, mô hình sẽ bị thiên vị nặng (Bias) hoặc học vẹt (Overfitting).
- **Giải pháp của Autoencoder:** Huấn luyện **HOÀN TOÀN TRÊN DỮ LIỆU LÁI XE BÌNH THƯỜNG (One-Class Learning)**.
  - Autoencoder học quy luật: "Thế nào là lái xe bình thường (chạy thẳng, cua, dừng đèn đỏ, gờ giảm tốc nhẹ)".
  - Khi xe gặp sự cố tai nạn (thứ nó chưa từng thấy bao giờ), mạng sẽ **KHÔNG THỂ TÁI TẠO ĐƯỢC** $\rightarrow$ Sai số tái tạo (Reconstruction Error) tăng vọt $\rightarrow$ Phát hiện dị thường!

```
Đầu vào x (4 đặc trưng)
     │
     ▼
[Encoder Layer 1]  (4 -> 8 neurons, LeakyReLU 0.1)
     │
     ▼
[Latent Bottleneck z] (8 -> 3 neurons, Tuyến tính)  <-- CỔ CHAI BIỂU DIỄN
     │
     ▼
[Decoder Layer 1]  (3 -> 8 neurons, LeakyReLU 0.1)
     │
     ▼
[Decoder Layer 2]  (8 -> 4 neurons, Tuyến tính)
     │
     ▼
Đầu ra tái tạo x̂ (4 đặc trưng)
```

### 4.2. Cơ chế Cổ chai (Bottleneck Compression: 4 $\rightarrow$ 8 $\rightarrow$ 3 $\rightarrow$ 8 $\rightarrow$ 4)
- Mạng nén 4 chiều đặc trưng đầu vào xuống **không gian ẩn chỉ có 3 chiều (Bottleneck $z \in \mathbb{R}^3$)**.
- Không gian 3 chiều này ép mạng phải loại bỏ các dao động ngẫu nhiên, chỉ giữ lại các mối tương quan nội tại của việc lái xe an toàn.
- **Hàm kích hoạt:** Sử dụng `LeakyReLU(0.1)` thay vì `ReLU` để giải quyết triệt để lỗi "chết nơ-ron" (Dying ReLU Problem), cho phép các gradient âm nhỏ tiếp tục lan truyền.

### 4.3. Hàm Sai số Tái tạo có trọng số (Weighted MSE Loss)
Sai số tái tạo giữa giá trị thực $x$ và giá trị mạng phục dựng $\hat{x}$:
$$\mathcal{L}_{recon} = \sum_{i=1}^{4} w_i \cdot \left(x_{norm}^{(i)} - \hat{x}_{norm}^{(i)}\right)^2$$
Bảng phân bổ trọng số vật lý:
- $w_{\text{Jerk}} = 0.40$ ($40\%$): Nhạy nhất với va chạm biến dạng cơ học.
- $w_{\text{RMS}} = 0.25$ ($25\%$): Đo xung năng lượng va đập.
- $w_{\text{Roll}} = 0.25$ ($25\%$): Đo góc lật ngã xe.
- $w_{\text{Pitch}} = 0.10$ ($10\%$): Đo góc chúi đầu/bốc đầu xe.

### 4.4. Cách xác định Ngưỡng Dị thường chuẩn thống kê (Statistical 3-Sigma Rule)
Không chọn ngưỡng theo cảm tính, mà dựa trên tập kiểm định Validation (chỉ gồm dữ liệu bình thường):
$$\text{Threshold} = \mu_{\text{val}} + 3 \cdot \sigma_{\text{val}} \approx 0.0497$$
Theo quy tắc phân phối chuẩn Gaussian, $99.73\%$ dữ liệu lái xe bình thường sẽ có sai số nằm dưới ngưỡng này. Bất cứ giá trị nào vượt ngưỡng đều là dị thường có ý nghĩa thống kê cao.

---

## 5. QUY TRÌNH HUẤN LUYỆN & ĐÓNG BĂNG TRỌNG SỐ (C EXPORT)

### 5.1. Quy trình huấn luyện Python (Zero Leakage)
1. **Thu thập dữ liệu:** Bộ dữ liệu chuẩn $F_s = 100$ Hz gồm các pha: Chạy thẳng $\rightarrow$ Ôm cua trái/phải $\rightarrow$ Qua ổ gà / gờ giảm tốc $\rightarrow$ Đâm va chạm $\rightarrow$ Đổ rạp $\rightarrow$ Phục hồi.
2. **Chia tập dữ liệu (Chronological Split):**
   - Tập Train ($80\%$ dữ liệu bình thường ban đầu).
   - Tập Validation ($20\%$ dữ liệu bình thường tiếp theo, độc lập hoàn toàn để chọn Threshold).
   - Tuyệt đối không xáo trộn ngẫu nhiên (No random shuffle) để tránh hiện tượng rò rỉ dữ liệu thời gian (Data Leakage).
3. **Huấn luyện:** Optimizer `Adam(lr=0.005, weight_decay=1e-5)` trong 80 Epochs.
4. **Hạ trọng số:** Trích xuất toàn bộ ma trận trọng số $W, B$ và tham số Min-Max ra file C Header: `tinyml_weights.h`.

### 5.2. Tối ưu hóa trên MCU ARM Cortex-M0 (Sonix SN32F407)
- Cortex-M0 **không có bộ FPU phần cứng** (Không có tập lệnh xử lý số thực dấu phẩy động bằng phần cứng).
- Để giải quyết:
  - Giữ kiến trúc mạng cực gọn: Tổng số phép nhân ma trận chỉ là $(4 \times 8) + (8 \times 3) + (3 \times 8) + (8 \times 4) = 112$ phép nhân cộng (MACs).
  - Không dùng thư viện cồng kềnh như TensorFlow Lite for Microcontrollers (TFLM ngốn $> 30$ KB Flash, vượt quá dung lượng ROM của chip).
  - Viết code C thuần bare-metal với các vòng lặp unroll đơn giản:
    - **Flash ROM chiếm dụng:** Chỉ **~560 Bytes** (chiếm $< 1.8\%$ ROM 32KB).
    - **SRAM chiếm dụng:** Chỉ **~88 Bytes** (chiếm $< 1.1\%$ RAM 8KB).
    - **Thời gian thực thi 1 lần suy luận:** Chỉ **1.1 mili-giây** tại xung nhịp 12 MHz!

---

## 6. BỘ LỌC XÁC NHẬN 2 GIAI ĐOẠN (TWO-STAGE LATCH STATE MACHINE)

Nếu chỉ dùng riêng mạng AI, khi xe sụp ổ gà cực sâu, xung kích có thể khiến sai số $MSE$ vượt ngưỡng trong chốc lát ($50 - 100$ ms). Để **triệt tiêu 100% báo động giả**, một máy trạng thái (FSM) 2 giai đoạn được tích hợp ngay sau đầu ra của AI:

```mermaid
stateDiagram-v2
    [*] --> IDLE : Xe chạy bình thường
    IDLE --> LATCHED : AI phát hiện MSE > Threshold (Có xung bất thường)
    
    state LATCHED {
        [*] --> CHECK_IMPACT
        CHECK_IMPACT --> SEVERE_CRASH : MSE > 5 * Threshold (Va chạm cực mạnh)
        CHECK_IMPACT --> MONITOR_WINDOW : MSE > Threshold (Mở cửa sổ 1.5s - 2.0s)
        
        MONITOR_WINDOW --> ACCIDENT_CONFIRMED : Góc |Roll| >= 45 độ liên tục >= 500ms
        MONITOR_WINDOW --> DISCARD_POTHOLE : Hết 2.0s mà xe vẫn đứng (|Roll| < 25 độ)
    }
    
    SEVERE_CRASH --> SOS_ALARM : Báo động khẩn cấp tức thì!
    ACCIDENT_CONFIRMED --> SOS_ALARM : Xác nhận tai nạn thực sự 100%!
    DISCARD_POTHOLE --> IDLE : Ổ gà / Xóc nẩy -> Xóa cờ, an toàn!
    SOS_ALARM --> IDLE : Gửi SMS cứu hộ thành công
```

### Các quy tắc logic chuẩn vật lý:
1. **Pha 1 (Bắt xung):** Khi $MSE > \text{Threshold}$, lập tức bật cờ nghi ngờ `s_anomaly_latched = 1` và kích hoạt bộ đếm thời gian cửa sổ ($0 \rightarrow 2000$ ms).
2. **Pha 2 (Kiểm chứng rơi đổ):**
   - **Trường hợp 1 (Severe Impact):** Nếu $MSE > 5 \times \text{Threshold}$ (va chạm trực diện cực kỳ thảm khốc) $\rightarrow$ Không cần đợi, báo động khẩn cấp ngay lập tức!
   - **Trường hợp 2 (Va chạm dẫn đến đổ xe):** Trong cửa sổ $1.5 - 2.0$ giây, nếu phát hiện góc xe bị nghiêng nằm sát đường ($|\text{Roll}| \ge 45^\circ$) duy trì liên tục $\ge 500$ ms $\rightarrow$ **XÁC NHẬN TAI NẠN 100%** $\rightarrow$ Bật còi báo động, nhấp nháy đèn và kích hoạt gửi SMS khẩn cấp!
   - **Trường hợp 3 (Ổ gà / Xóc nẩy):** Sau $2.0$ giây, xe vẫn lấy lại được thăng bằng ($|\text{Roll}| < 25^\circ$) $\rightarrow$ Tự động xóa cờ cảnh báo, quay về trạng thái IDLE bình thường. Không gửi SMS nhầm!

---

## 7. PHÂN CÔNG NHIỆM VỤ PHẦN CỨNG: FPGA VS. MCU SONIX

Một điểm sáng tạo then chốt của đề tài là kiến trúc cộng tác dị thể **(Heterogeneous FPGA + MCU Co-Processing)**:

```
[MPU-6050] ──I2C──> [FPGA GOWIN GW1NSR-4C] ──WAKE IRQ (Pin 41)──> [MCU SONIX SN32F407]
                             │                                             │
                             └─── SPI Master (64-bit Payload) ─────────────┤
                                  (Pitch, Roll, RMS, Jerk)                 ├──> GPS NEO-M6N (UART0)
                                                                           └──> 4G SIM A7680C (UART1)
```

| Tiêu chí | FPGA Gowin GW1NSR-4C | MCU Sonix SN32F407 |
| :--- | :--- | :--- |
| **Vai trò chính** | Tiền xử lý phần cứng tốc độ cao (Ultra Low-Power Watchdog) | Trí tuệ nhân tạo TinyML & Điều phối viễn thông cứu hộ |
| **Nhiệm vụ cụ thể** | - Giao tiếp I2C đọc MPU-6050 liên tục.<br>- Lọc chống dội nút bấm K1 (Debounce 20ms).<br>- Phát xung ngắt WAKE IRQ cực nhanh ($< 20$ ns).<br>- Đẩy gói tin 64-bit đặc trưng qua SPI Master. | - Nhận ngắt thức dậy qua chân P2.9.<br>- Đọc gói 64-bit từ SPI Slave FIFO.<br>- Chạy mô hình suy luận TinyML Autoencoder.<br>- Lọc trạng thái Two-Stage Latch.<br>- Tách câu NMEA GPS từ UART0.<br>- Điều khiển modem 4G gửi tin SMS đến `0792394699`. |
| **Công suất tiêu thụ** | Cực thấp (Micro-Ampe khi chạy logic tĩnh) | Tiết kiệm pin tối đa (Chỉ thức dậy khi có sự cố) |

---

## 8. BỘ 15 CÂU HỎI VẤN ĐÁP "TỬ HUYỆT" & CÁCH TRẢ LỜI ĐANH THÉP TRƯỚC HỘI ĐỒNG

### Câu 1: Tại sao nhóm không dùng mạng nơ-ron tích chập (CNN) hay LSTM mà lại dùng Autoencoder?
> **Trả lời:**  
> - *Thứ nhất về tài nguyên phần cứng:* Vi điều khiển Sonix SN32F407 là dòng ARM Cortex-M0 chỉ có 32KB Flash và 8KB SRAM, không có FPU. Các mạng CNN hay LSTM đòi hỏi hàng trăm Kilobytes bộ nhớ và hàng triệu phép tính dấu phẩy động, hoàn toàn không thể chạy bare-metal trên dòng chip này.
> - *Thứ hai về bản chất bài toán:* Dữ liệu tai nạn cực kỳ mất cân bằng (tai nạn chỉ chiếm $< 0.001\%$). CNN/LSTM phân loại nhị phân sẽ bị học vẹt hoặc sai lệch lớn. Autoencoder là mô hình học không giám sát (Unsupervised), chỉ cần học dữ liệu lái xe bình thường và phát hiện bất thường dựa trên sai số tái tạo, giải quyết trọn vẹn bài toán thiếu dữ liệu tai nạn thực tế.

---

### Câu 2: Sai số tái tạo (Reconstruction Error) được tính như thế nào và có ý nghĩa gì?
> **Trả lời:**  
> Sai số tái tạo là chênh lệch bình phương giữa vectơ đầu vào $x$ và vectơ do mạng phục dựng $\hat{x}$. Khi xe chạy bình thường, mạng đã học thuộc quy luật nên tái tạo lại gần như trùng khớp ($MSE \approx 0$). Nhưng khi có tai nạn, các xung gia tốc và Jerk tăng vọt vượt khỏi miền phân bố bình thường, mạng không thể tái tạo được khiến sai số $MSE$ tăng vọt lên gấp hàng chục lần.

---

### Câu 3: Làm thế nào nhóm chọn được con số Ngưỡng dị thường (Threshold = 0.0497)?
> **Trả lời:**  
> Ngưỡng không được chọn mò mẫm hay cảm tính, mà dựa trên phương pháp thống kê toán học chuẩn: **Nguyên lý 3-Sigma ($\mu + 3\sigma$)**. Nhóm lấy giá trị trung bình $\mu$ cộng với 3 lần độ lệch chuẩn $\sigma$ của sai số tái tạo trên tập dữ liệu kiểm định (Validation) độc lập. Theo phân phối chuẩn, ngưỡng này đảm bảo $99.73\%$ các dao động lái xe bình thường sẽ không bao giờ bị kích hoạt nhầm.

---

### Câu 4: Xe đi qua ổ gà lớn hoặc gờ giảm tốc rất xóc, làm sao hệ thống của nhóm không gửi tin nhắn báo động giả?
> **Trả lời:**  
> Nhờ vào **Bộ lọc xác nhận 2 giai đoạn (Two-Stage Latch Confirmation)**:
> - Ổ gà dù có xóc mạnh đến đâu thì thời gian xung kích chỉ diễn ra trong $50 - 150$ mili-giây và xe vẫn giữ được thăng bằng ($|\text{Roll}| < 25^\circ$).
> - Khi AI phát hiện xung ổ gà, hệ thống chưa gửi tin nhắn ngay mà chỉ chốt cờ nghi ngờ và mở cửa sổ quan sát $1.5 - 2.0$ giây. Sau 2 giây, thấy xe vẫn đứng thẳng, hệ thống tự động hủy cờ, triệt tiêu $100\%$ việc báo giả. Tin nhắn chỉ gửi đi khi trong cửa sổ đó xe bị ngã đổ nghiêng rạp ($|\text{Roll}| \ge 45^\circ$) duy trì liên tục trên 500ms.

---

### Câu 5: Tại sao xe ôm cua nghiêng tới 35 - 40 độ mà không bị hệ thống nhận diện là tai nạn?
> **Trả lời:**  
> - Khi ôm cua bình thường, việc nghiêng xe diễn ra mượt mà theo lực hướng tâm, không đi kèm xung kích va chạm biến dạng cơ học, do đó đại lượng **Dynamic RMS** và **Vector Jerk** vẫn ở mức rất thấp.
> - Autoencoder của nhóm sử dụng hàm sai số có trọng số đặt $40\%$ vào Jerk và $25\%$ vào RMS. Do không có Jerk đột biến, sai số $MSE$ vẫn nằm an toàn dưới ngưỡng.

---

### Câu 6: Đại lượng "3D Vector Jerk" là gì và tại sao nó lại quan trọng hơn Gia tốc thông thường?
> **Trả lời:**  
> Jerk là đạo hàm bậc nhất của gia tốc theo thời gian ($Jerk = \frac{da}{dt}$, đơn vị $g/s$). Trong vật lý, Jerk thể hiện tốc độ tăng của lực tác động. Khi xe qua ổ gà, phuộc nhún sẽ triệt tiêu xung lực nên Jerk tăng có giới hạn ($< 50 g/s$). Nhưng khi có va chạm cơ học với vật cứng (đâm xe, tông dải phân cách), lực tác động tăng vọt chỉ trong vài mili-giây khiến Jerk vọt lên tới $150 - 400 g/s$. Vì vậy, Jerk là "chìa khóa vàng" để nhận diện va chạm phá hủy.

---

### Câu 7: Bộ lọc FIR 15-tap đóng vai trò gì? Tại sao phải cắt ở 5Hz?
> **Trả lời:**  
> Tần số cắt 5Hz được chọn dựa trên phổ phân tích FFT thực tế:
> - Dao động tư thế cơ thể và nghiêng xe của người lái luôn nằm ở dải tần thấp dưới $3 - 5$ Hz.
> - Rung động cơ xe máy 4 thì nằm ở dải tần $30 - 45$ Hz.
> - Bộ lọc FIR thông thấp 5Hz sẽ triệt tiêu hoàn toàn rung động cơ và nhiễu vi chấn mặt đường, giữ lại vectơ trọng trường $1.0g$ thuần khiết để tính góc Pitch và Roll chính xác tuyệt đối mà không bị rung giật số.

---

### Câu 8: Tại sao lại cần cả FPGA và MCU? Sao không để MCU làm hết từ đầu đến cuối?
> **Trả lời:**  
> Đây là kiến trúc tối ưu hóa năng lượng và độ tin cậy chuẩn công nghiệp:
> - MCU Sonix nếu phải liên tục lấy mẫu I2C 100Hz và tính toán toán học thì CPU sẽ luôn chạy ở 100% công suất, gây nóng chip và nhanh cạn ắc-quy xe máy.
> - FPGA Gowin là phần cứng song song tiêu thụ điện cực thấp. FPGA đóng vai trò là "người lính gác" (Hardware Watchdog) thường trực. Chỉ khi có sự cố, FPGA mới phát xung WAKE IRQ đánh thức MCU dậy xử lý AI và viễn thông. Điều này giúp hệ thống hoạt động bền bỉ nhiều tháng trên xe.

---

### Câu 9: Cơ chế truyền dữ liệu giữa FPGA và MCU diễn ra như thế nào?
> **Trả lời:**  
> Truyền đồng bộ qua chuẩn **SPI Mode 0 (CPOL=0, CPHA=0)** ở tốc độ 270 kHz:
> - FPGA hoạt động ở chế độ SPI Master, MCU Sonix hoạt động ở chế độ Hardware SPI Slave.
> - Gói tin gồm 64-bit (8 bytes), phân rã thành 4 từ 16-bit: `{Pitch, Roll, Dynamic RMS, Jerk}`. Toàn bộ 4 từ này được nạp thẳng vào bộ đệm phần cứng 8-word RX FIFO của chip Sonix SN32F407, không làm rơi rớt bất kỳ bit dữ liệu nào.

---

### Câu 10: Nhóm xử lý bài toán bộ nhớ trên chip Sonix SN32F407 (RAM 8KB, Flash 32KB) như thế nào?
> **Trả lời:**  
> - Không dùng bất kỳ thư viện chuẩn nặng nề nào như `<stdio.h>`, `<string.h>` hay `printf` của runtime chuẩn.
> - Toàn bộ mã nguồn C được viết bare-metal thao tác trực tiếp trên thanh ghi phần cứng (Direct Register Manipulation).
> - Mô hình Autoencoder được unroll các vòng lặp nhân ma trận trực tiếp, mảng trọng số khai báo dưới dạng `static const` để nằm cố định trên Flash ROM thay vì chiếm dụng RAM. Nhờ đó, chương trình chỉ chiếm ~17KB Flash và ~1.7KB RAM.

---

### Câu 11: Nếu SIM mất sóng hoặc chưa gửi được tin nhắn thì hệ thống xử lý ra sao?
> **Trả lời:**  
> Trong hàm `Execute_Emergency_Routine`, nhóm triển khai vòng lặp cứu hộ **"Thực hiện đến khi thành công" (`while (!sms_ok)`)**:
> - Nếu modem trả về lỗi hoặc timeout, MCU tự động thoát phiên bằng mã `ESC (0x1B)`, đợi 2 giây và thử gửi lại.
> - Tin nhắn được gửi song song ở cả 2 định dạng: Quốc tế `+84792394699` và Nội địa `0792394699` để tương thích mọi trạm BTS.
> - Đồng thời, còi chip và đèn cảnh báo đỏ sẽ duy trì liên tục cho đến khi nhận được mã phản hồi `+CMGS: ... OK` từ nhà mạng.

---

### Câu 12: Tại sao trong nội dung tin nhắn SMS khẩn cấp, nhóm không chèn đường link Google Maps (https://maps.google.com/?q=...)?
> **Trả lời:**  
> Đây là kinh nghiệm thực chiến viễn thông tại Việt Nam: Theo **Nghị định 91/2020/NĐ-CP** về chống tin nhắn rác và lừa đảo, các nhà mạng lớn (Viettel, MobiFone, VinaPhone) đã kích hoạt bộ lọc tự động chặn (drop) toàn bộ tin nhắn cá nhân có chứa đường dẫn URL (http/https). Do đó, nhóm chỉ gửi văn bản thuần kèm tọa độ số nguyên bản (`Toa do: 16.078577,108.150899`). Người nhận chỉ cần chạm vào tọa độ là điện thoại tự động mở bản đồ dẫn đường.

---

### Câu 13: Mô hình TinyML của nhóm mất bao lâu để đưa ra quyết định từ khi va chạm xảy ra?
> **Trả lời:**  
> - Thời gian tính toán tiền xử lý trên FPGA: $< 1$ mili-giây.
> - Thời gian truyền gói 64-bit qua SPI: $\approx 1.5$ mili-giây.
> - Thời gian suy luận Autoencoder trên MCU: $\approx 1.1$ mili-giây.
> - Cửa sổ kiểm chứng đổ xe vật lý: $500$ mili-giây.
> $\Rightarrow$ **Tổng thời gian ra quyết định chính xác: $\approx 0.5 - 0.6$ giây**, nhanh hơn phản xạ nhận thức của con người và kịp thời trước khi nạn nhân rơi vào trạng thái mất ý thức.

---

### Câu 14: Tại sao trong code Verilog của FPGA lại có đoạn trễ 500ms khi vừa cắm điện (Power-on Lock)?
> **Trả lời:**  
> Khi vừa bật khóa điện xe máy hoặc cấp nguồn, các tụ điện trên bo mạch và cảm biến IMU MPU-6050 trải qua giai đoạn quá độ nạp xả dòng (Inrush Current), tạo ra các xung áp giả (Glitches). Đoạn khóa 500ms (`power_on_lock`) đảm bảo toàn bộ hệ thống nguồn đã ổn định xác lập trước khi cho phép nút bấm và mạch ngắt hoạt động, chống kích hoạt giả khi khởi động xe.

---

### Câu 15: Nếu ban giám khảo hỏi: "Nếu xe bị va chạm nhưng không đổ (ví dụ đâm trực diện ô tô ở tốc độ cao rồi dính chặt vào đầu xe), hệ thống có phát hiện được không?"
> **Trả lời:**  
> **Chắc chắn phát hiện được 100%!**  
> Trong hàm `tinyml_confirm_accident()`, nhóm đã lập trình nhánh rẽ ưu tiên:
> ```c
> if (mse_score > (5.0f * RECON_THRESHOLD)) {
>     return 1; /* Báo động khẩn cấp tức thì! */
> }
> ```
> Khi va chạm trực diện cực mạnh, xung gia tốc và Jerk vọt lên đỉnh điểm khiến sai số tái tạo $MSE$ vượt gấp 5 lần ngưỡng dị thường. Trong tình huống này, hệ thống nhận diện đây là tai nạn nguy kịch và kích hoạt báo động cứu hộ ngay lập tức mà không cần kiểm tra điều kiện xe có bị đổ rạp hay không.

---

## TỔNG KẾT BẢNG THÔNG SỐ VÀNG (CHEAT-SHEET ĐỂ NHỚ NHANH)

| Thông số | Giá trị chuẩn | Ý nghĩa kỹ thuật |
| :--- | :--- | :--- |
| **Tần số lấy mẫu ($F_s$)** | $100$ Hz | Chu kỳ 10ms, thỏa mãn tiêu chuẩn Nyquist |
| **Tần số cắt FIR ($F_c$)** | $5.0$ Hz | 15 Taps, triệt tiêu rung động cơ 30-45Hz |
| **Kiến trúc Mạng** | $4 \rightarrow 8 \rightarrow 3 \rightarrow 8 \rightarrow 4$ | Autoencoder đối xứng với Bottleneck 3 chiều |
| **Hàm kích hoạt** | LeakyReLU ($\alpha = 0.1$) | Chống hiện tượng chết nơ-ron (Dying ReLU) |
| **Trọng số sai số ($w_i$)** | Jerk: 40%, RMS: 25%, Roll: 25%, Pitch: 10% | Tối ưu hóa độ nhạy va chạm phá hủy |
| **Ngưỡng dị thường** | $0.0497$ | Tính theo nguyên lý $3\sigma$ ($\mu + 3\sigma$) |
| **Bộ nhớ Flash MCU** | $560$ Bytes | Tối ưu bare-metal, unroll vòng lặp |
| **Thời gian suy luận** | $1.1$ ms | Chạy trên lõi ARM Cortex-M0 @ 12MHz |
| **Số điện thoại mục tiêu** | `0792394699` / `+84792394699` | Gửi qua module LTE A7680C |

---
*Tài liệu được biên soạn đồng bộ trực tiếp với mã nguồn dự án: `main.c`, `tinyml_infer.c`, `fpga_trigger_top.v` và `ai_pipeline_full.py`.*
