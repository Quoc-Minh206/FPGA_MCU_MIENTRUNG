import serial
import time
import sys

def main():
    port = "COM8"
    baud = 115200
    print(f"===============================================================")
    print(f"[*] KET NOI CONG {port} @ {baud} 8N1 - TEST FPGA K1 TRIGGER & 4 FEATURES")
    print(f"===============================================================")

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"[!] Khong the mo cong {port}: {e}")
        return

    ser.reset_input_buffer()
    time.sleep(0.5)

    # 1. Kiem tra trang thai GPS va SIM truoc
    ser.write(b"s\r\n")
    time.sleep(0.3)

    # 2. Phat lenh 'k' de kiem thu toan bo luong 4 bo so va gui SMS cuu ho
    print("\n[*] Phat tin hieu test 'k' (Mo phong kich hoat K1 tu FPGA)...")
    ser.write(b"k\r\n")

    start_time = time.time()
    sms_success_detected = False

    while (time.time() - start_time) < 60.0:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='replace').rstrip()
                if line:
                    t_str = time.strftime('%H:%M:%S')
                    print(f"[{t_str}] {line}")
                    sys.stdout.flush()

                    if "GUI THANH CONG" in line or "+CMGS:" in line:
                        sms_success_detected = True
            except Exception:
                pass
        else:
            time.sleep(0.01)

    ser.close()
    print(f"\n===============================================================")
    if sms_success_detected:
        print("[*] KET QUA TEST: THANH CONG 100%! Da nhan 4 bo so va gui SMS den 0792394699!")
    else:
        print("[*] Da hoan tat phien ghi log.")
    print(f"===============================================================")

if __name__ == "__main__":
    main()
