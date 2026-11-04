import serial
import time
import sys

def main():
    port = "COM8"
    baud = 115200
    print(f"[*] Mo cong {port} o toc do {baud} 8N1 de theo doi...")

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"[!] Khong the mo cong {port}: {e}")
        return

    # Doc can bo dem cu
    ser.reset_input_buffer()
    
    # Gui lenh 's' de xem trang thai hien tai
    time.sleep(0.5)
    ser.write(b"s\r\n")
    
    print("[*] Bat dau lang nghe su kien tu FPGA (Nut K1) tren UART COM8...")
    print("[*] (Script se theo doi trong 45 giay hoac den khi hoan thanh chu trinh)\n")

    start_time = time.time()
    last_print = time.time()

    while (time.time() - start_time) < 45.0:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='replace').rstrip()
                if line:
                    print(f"[{time.strftime('%H:%M:%S')}] {line}")
                    sys.stdout.flush()
            except Exception:
                pass
        else:
            time.sleep(0.01)

    ser.close()
    print("\n[*] Ket thuc phien theo doi.")

if __name__ == "__main__":
    main()
