import serial
import time
import sys

PORT = 'COM10'
BAUD = 115200
LOG_FILE = r'C:\Users\Lenovo\Downloads\main_project\uart_live.log'

print(f"=== Starting Real-time Serial Logger on {PORT} at {BAUD} ===")
sys.stdout.flush()

try:
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        with open(LOG_FILE, 'a', encoding='utf-8') as f:
            f.write(f"\n========================================\n")
            f.write(f"=== Serial Logger started at {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            f.write(f"========================================\n")
            f.flush()

        print(f"[LOGGER] Listening... Ready to receive data from MCU.")
        sys.stdout.flush()

        while True:
            raw = ser.readline()
            if raw:
                try:
                    line = raw.decode('utf-8', errors='replace').rstrip()
                except Exception:
                    line = str(raw)
                timestamp = time.strftime("[%H:%M:%S]")
                formatted = f"{timestamp} {line}"
                print(formatted)
                sys.stdout.flush()
                with open(LOG_FILE, 'a', encoding='utf-8') as f:
                    f.write(formatted + "\n")
                    f.flush()
except Exception as e:
    print(f"[LOGGER ERROR]: {e}")
    sys.stdout.flush()
