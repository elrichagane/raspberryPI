import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# Wait for READY line
t0 = time.time()
while True:
    line = ser.readline().decode(errors='ignore').strip()
    if line.startswith("READY"):
        break
    if time.time() - t0 > 5:
        raise RuntimeError("Arduino not responding")

print("Arduino ready:", line)

while True:
    line = ser.readline().decode(errors='ignore').strip()
    if not line:
        continue
    # Example line: PULSE,12,12345678,4294987654321
    if line.startswith("PULSE"):
        parts = line.split(',')
        seq = int(parts[1])
        t32 = int(parts[2])
        t64 = int(parts[3])
        print(seq, t32, t64)