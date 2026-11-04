import serial
import subprocess
import time

def main():
    print("=== 1. Opening COM8 @ 115200 ===")
    try:
        ser = serial.Serial('COM8', 115200, timeout=0.1)
        print("COM8 opened successfully.")
    except Exception as e:
        print("Failed to open COM8:", e)
        return

    print("=== 2. Building & Flashing Firmware via Keil SN-Link ===")
    cmd = [
        "powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-Command", "Set-Location 'C:\\Users\\Lenovo\\Downloads\\main_project'; .\\build_and_flash.ps1"
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    print("Flash Output Summary:")
    for line in res.stdout.splitlines():
        if "Load" in line or "Verify OK" in line or "finished" in line or "Complete" in line or "Error" in line:
            print("  [FLASH]", line)

    print("=== 3. Monitoring COM8 Live Serial Output for 65s (Call + SMS Test) ===")
    t0 = time.time()
    while time.time() - t0 < 65:
        line = ser.readline()
        if line:
            try:
                decoded = line.decode('utf-8', errors='replace').rstrip()
                if decoded:
                    print(f"[{time.time()-t0:4.1f}s] {decoded}")
            except Exception:
                pass
        time.sleep(0.005)

    ser.close()
    print("=== 4. Test Complete! COM8 closed. ===")

if __name__ == '__main__':
    main()
