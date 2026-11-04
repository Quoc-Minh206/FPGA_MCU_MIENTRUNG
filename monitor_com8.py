import serial, time, sys

sys.stdout.reconfigure(encoding='utf-8')
duration = 45
if len(sys.argv) > 1:
    duration = int(sys.argv[1])

print(f"=== Opening COM8 @ 115200 (Duration: {duration}s) ===", flush=True)
try:
    s = serial.Serial('COM8', 115200, timeout=0.5)
    start = time.time()
    while (time.time() - start) < duration:
        line = s.readline()
        if line:
            text = line.decode('utf-8', errors='replace').rstrip()
            elapsed = time.time() - start
            print(f"[{elapsed:5.1f}s] {text}", flush=True)
    s.close()
    print("=== Monitor Finished ===", flush=True)
except Exception as e:
    print(f"Error opening COM8: {e}", flush=True)
