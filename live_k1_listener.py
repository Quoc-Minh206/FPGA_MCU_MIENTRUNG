import serial
import time
import sys

def main():
    port = "COM8"
    baud = 115200
    log_file = "fpga_k1_test_log.txt"

    print("=================================================================")
    print(f"[*] DANG LANG NGHE SU KIEN K1 TU FPGA TREN {port} @ {baud}")
    print(f"[*] (Ban co the nhan nut K1 hoac S2 tren bo FPGA bat cu luc nao)")
    print("=================================================================\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"[!] Khong the mo cong {port}: {e}")
        return

    ser.reset_input_buffer()

    with open(log_file, "a", encoding="utf-8") as f:
        f.write(f"\n--- PHIEN THEO DOI DAEMON: {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
        f.flush()

        start_time = time.time()
        # Chay lien tuc trong 600 giay (10 phut)
        while (time.time() - start_time) < 600.0:
            if ser.in_waiting > 0:
                try:
                    raw = ser.readline()
                    line = raw.decode('utf-8', errors='replace').rstrip()
                    if line:
                        timestamp = time.strftime('%H:%M:%S')
                        log_line = f"[{timestamp}] {line}"
                        print(log_line)
                        sys.stdout.flush()
                        f.write(log_line + "\n")
                        f.flush()

                        if "GUI THANH CONG" in line:
                            print("\n>>> [HOAN THANH] XAC NHAN GUI THANH CONG DEN SO 0792394699! <<<\n")
                            sys.stdout.flush()
                except Exception as ex:
                    pass
            else:
                time.sleep(0.01)

    ser.close()
    print("[*] Ket thuc phien lang nghe 10 phut.")

if __name__ == "__main__":
    main()
