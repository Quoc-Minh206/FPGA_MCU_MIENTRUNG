import serial
import time
import sys

def main():
    port = "COM8"
    baud = 115200
    log_file = "fpga_k1_test_log.txt"

    print("=================================================================")
    print(f"[*] DANG LANG NGHE SU KIEN K1 TU FPGA TREN {port} @ {baud}")
    print(f"[*] (Ho tro tu dong ket noi lai neu cong COM bi ban hoac ngat nhe)")
    print("=================================================================\n")

    ser = None
    start_time = time.time()

    with open(log_file, "a", encoding="utf-8") as f:
        f.write(f"\n--- PHIEN THEO DOI DAEMON: {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
        f.flush()

        while (time.time() - start_time) < 600.0:
            if ser is None:
                try:
                    ser = serial.Serial(port, baud, timeout=0.1)
                    ser.reset_input_buffer()
                    print(f"[{time.strftime('%H:%M:%S')}] Da ket noi {port} thanh cong.")
                    sys.stdout.flush()
                except Exception as e:
                    time.sleep(1.0)
                    continue

            try:
                if ser.in_waiting > 0:
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
                else:
                    time.sleep(0.01)
            except Exception as e:
                # Neu cong com bi dong hoac reset, dong va thu ket noi lai
                try:
                    if ser:
                        ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(1.0)

        if ser:
            ser.close()

if __name__ == "__main__":
    main()
