import serial
import time
import sys

def main():
    ser = serial.Serial("COM8", 115200, timeout=0.1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("[*] 1. Kiem tra trang thai he thong...")
    ser.write(b"s\r\n")
    time.sleep(0.5)

    print("[*] 2. Phat lenh kich hoat test mo phong FPGA K1 & 4 Bo so...")
    for _ in range(5):
        ser.write(b"k\r\n")
        time.sleep(0.04)

    t0 = time.time()
    while time.time() - t0 < 35.0:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line:
                print(f"[{time.strftime('%H:%M:%S')}] {line}")
                sys.stdout.flush()
        else:
            time.sleep(0.01)

    ser.close()
    print("[*] Hoan tat chu trinh.")

if __name__ == "__main__":
    main()
