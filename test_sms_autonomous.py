import serial, time, sys

sys.stdout.reconfigure(encoding='utf-8')

print("==========================================================")
print("   CHUONG TRINH TU DONG KIEM THU GUI SMS LIEN TIEP 4G")
print("   Cong: COM8 | Baudrate: 115200 | Nguoi nhan: 0792398699")
print("==========================================================")

try:
    s = serial.Serial('COM8', 115200, timeout=0.2)
except Exception as e:
    print(f"Loi mo cong COM8: {e}")
    sys.exit(1)

def read_lines_until(pattern, timeout_s=15):
    t0 = time.time()
    collected = []
    matched = False
    while time.time() - t0 < timeout_s:
        line = s.readline()
        if line:
            text = line.decode('utf-8', errors='replace').rstrip()
            if text:
                elapsed = time.time() - t0
                print(f"[{elapsed:5.1f}s] {text}", flush=True)
                collected.append(text)
                if pattern and pattern in text:
                    matched = True
                    break
    return matched, collected

# 1. Thu thập dữ liệu ban đầu
print("\n--- BƯỚC 1: Kiem tra trang thai MCU va ket noi SIM ---")
s.write(b'c')
matched, log = read_lines_until("da dang ky mang", 10)

# 2. Kích hoạt test gửi tin nhắn LẦN 1
print("\n--- BƯỚC 2: Kich hoat gui SMS LAN 1 den 0792398699 ---")
time.sleep(1)
s.write(b't')
matched1, log1 = read_lines_until("SUCCESS (3 beeps emitted)", 30)

if matched1:
    print("\n>>> KET QUA LAN 1: THANH CONG! SMS da duoc gui vao mang di dong! <<<")
else:
    print("\n>>> KET QUA LAN 1: Chua nhan duoc xac nhan hoan tat <<<")

# Chờ 5 giây trước khi test lần 2
print("\n--- Cho 5 giay giua 2 lan gui de mang on dinh ---")
time.sleep(5)

# 3. Kích hoạt test gửi tin nhắn LẦN 2 (xác minh lặp lại không bị kẹt)
print("\n--- BƯỚC 3: Kich hoat gui SMS LAN 2 (kiem tra lap lai) ---")
s.write(b't')
matched2, log2 = read_lines_until("SUCCESS (3 beeps emitted)", 30)

if matched2:
    print("\n>>> KET QUA LAN 2: THANH CONG! He thong lap lai hoan hao khong bi ket! <<<")
else:
    print("\n>>> KET QUA LAN 2: Chua nhan duoc xac nhan hoan tat <<<")

s.close()
print("\n==========================================================")
print(f"TONG KET KIEM THU: Lan 1 = {'DAT' if matched1 else 'CHUA DAT'} | Lan 2 = {'DAT' if matched2 else 'CHUA DAT'}")
print("==========================================================")
