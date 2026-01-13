"""
Test Unitree L2 LiDAR Connection
This script tests if we can receive UDP packets from the sensor.
"""

import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

print("=" * 70)
print("UNITREE L2 LIDAR CONNECTION TEST")
print("=" * 70)
print()
print("Sensor IP: 192.168.1.62:6101")
print("Host IP:   192.168.1.2:6201")
print()

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('192.168.1.2', 6201))
sock.settimeout(5.0)

print("Listening for data from Unitree L2...")
print("(Make sure sensor is powered on and configured to send to 192.168.1.2:6201)")
print()

try:
    # Try to receive a packet
    for i in range(10):
        print(f"Attempt {i+1}/10...", end=" ")
        try:
            data, addr = sock.recvfrom(4096)
            print(f"SUCCESS!")
            print()
            print(f"Received {len(data)} bytes from {addr}")
            print(f"First 100 bytes (hex): {data[:100].hex()}")
            print()
            print("Connection verified! Sensor is sending data.")
            break
        except socket.timeout:
            print("timeout")
            time.sleep(0.5)
    else:
        print()
        print("No data received after 10 attempts.")
        print()
        print("Troubleshooting:")
        print("1. Ensure sensor is powered on")
        print("2. Check sensor configuration (should send to 192.168.1.2:6201)")
        print("3. Verify firewall allows UDP port 6201")
        print("4. Confirm sensor is set to 3D point packet mode")

except KeyboardInterrupt:
    print("\nTest cancelled by user")
finally:
    sock.close()
    print()
    print("=" * 70)