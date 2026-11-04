import serial
import time
import sys

def main():
    ser = serial.Serial("COM8", 115200, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("[*] Gui xung lenh 'k' den MCU...")
    for _ in range(5):
        ser.write(b"k\r\n")
        time.sleep(0.05)

    t0 = time.time()
    while time.time() - t0 < 30.0:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line:
                print(f"[{time.strftime('%H:%M:%S')}] {line}")
                sys.stdout.flush()
        else:
            time.sleep(0.01)

    ser.close()

if __name__ == "__main__":
    main()
