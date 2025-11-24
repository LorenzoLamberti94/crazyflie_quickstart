import serial
import struct
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# Hex
# MESSAGE = b"\x00\x00\x00\x01\x00\x00\x00\x01" # 8 bytes message  # BE CAREFUL: this is big-endian, and the Crazyflie expects little-endian. Therefore, you will see bytes in reverse order.

# int32
MESSAGE = struct.pack("<ii", 1, 2) # 4 bytes message  ## this reads correctly on the Crazyflie side, no little/big endian problems.

# float32
# MESSAGE = struct.pack("<ff", 1.0, 2.0) #


def check_usb_device(port="/dev/ttyUSB0", watch_time=3):
    import os
    import subprocess

    print(f"Checking for {port} ...")

    # 1️⃣ Check if the device exists
    if not os.path.exists(port):
        print(f"[ERROR] {port} not found.")
        print("Recent dmesg logs:")
        subprocess.run("dmesg | tail", shell=True)
        return False

    print(f"[OK] {port} detected.")

check_usb_device(PORT)
ser = serial.Serial (PORT)
ser.baudrate = BAUDRATE

print(f"Opened serial port {PORT} at {BAUDRATE} baud.")
while True:
    # print(MESSAGE)
    print(MESSAGE[::-1]) # Print reversed bytes for clarity
    ser.write(MESSAGE)
    time.sleep(33/1000)

ser.close()