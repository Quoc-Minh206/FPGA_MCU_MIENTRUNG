import serial
import time

def trigger_sms():
    try:
        ser = serial.Serial('COM8', 115200, timeout=1)
        print("COM8 opened. Sending 't' command to trigger SMS...")
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.write(b't\r\n')
        ser.flush()

        t0 = time.time()
        while time.time() - t0 < 15:
            line = ser.readline()
            if line:
                print(line.decode('utf-8', errors='replace').strip())
        ser.close()
    except Exception as e:
        print("Error:", e)

if __name__ == '__main__':
    trigger_sms()
