"""
Debug script to analyze raw IMU packet data from Unitree L2 LiDAR.
This script will dump the raw bytes to understand the actual packet structure.
"""

import sys
from pathlib import Path
import socket
import struct
import time

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Protocol constants
FRAME_HEADER = bytes([0x55, 0xAA, 0x05, 0x0A])
LIDAR_POINT_DATA_PACKET_TYPE = 102
LIDAR_2D_POINT_DATA_PACKET_TYPE = 103
LIDAR_IMU_DATA_PACKET_TYPE = 104


def hex_dump(data: bytes, bytes_per_line: int = 16) -> str:
    """Format bytes as hex dump."""
    lines = []
    for i in range(0, len(data), bytes_per_line):
        chunk = data[i:i+bytes_per_line]
        hex_part = ' '.join(f'{b:02x}' for b in chunk)
        ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f'{i:04x}: {hex_part:<48} {ascii_part}')
    return '\n'.join(lines)


def parse_imu_multiple_offsets(payload: bytes):
    """Try parsing IMU data at various offsets to find the quaternion."""
    print(f"\nPayload length: {len(payload)} bytes")
    print("\nFirst 128 bytes of payload (hex dump):")
    print(hex_dump(payload[:128]))

    print("\n" + "="*60)
    print("ATTEMPTING QUATERNION PARSING AT VARIOUS OFFSETS")
    print("="*60)

    # Try different offsets
    for offset in [0, 4, 8, 12, 16, 20, 24, 28, 32]:
        if offset + 16 > len(payload):
            continue
        try:
            qw = struct.unpack('<f', payload[offset:offset+4])[0]
            qx = struct.unpack('<f', payload[offset+4:offset+8])[0]
            qy = struct.unpack('<f', payload[offset+8:offset+12])[0]
            qz = struct.unpack('<f', payload[offset+12:offset+16])[0]
            norm = (qw*qw + qx*qx + qy*qy + qz*qz) ** 0.5

            # Check if it looks like a valid quaternion (norm close to 1)
            valid = 0.9 < norm < 1.1
            marker = " <-- VALID QUATERNION!" if valid else ""

            print(f"\nOffset {offset:2d}: qw={qw:8.4f}, qx={qx:8.4f}, qy={qy:8.4f}, qz={qz:8.4f} | norm={norm:.4f}{marker}")
        except Exception as e:
            print(f"Offset {offset}: Error - {e}")

    # Also try parsing as raw floats to see what values are in the payload
    print("\n" + "="*60)
    print("ALL FLOATS IN FIRST 64 BYTES")
    print("="*60)

    for i in range(0, min(64, len(payload)), 4):
        if i + 4 <= len(payload):
            val = struct.unpack('<f', payload[i:i+4])[0]
            # Mark values that could be quaternion components (-1 to 1)
            marker = " *" if -1.1 < val < 1.1 and abs(val) > 0.001 else ""
            print(f"  [{i:2d}:{i+4:2d}] = {val:12.6f}{marker}")


def main():
    print("="*60)
    print("  UNITREE L2 IMU PACKET DEBUGGER")
    print("="*60)

    # Setup UDP socket
    host_ip = "192.168.1.2"
    host_port = 6201

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host_ip, host_port))
    sock.settimeout(5.0)

    print(f"\nListening on {host_ip}:{host_port}")
    print("Waiting for IMU packets...\n")

    packet_counts = {102: 0, 103: 0, 104: 0}
    imu_packets_analyzed = 0
    max_imu_packets = 5

    start_time = time.time()
    timeout = 30  # seconds

    try:
        while time.time() - start_time < timeout:
            try:
                data, addr = sock.recvfrom(8192)
            except socket.timeout:
                print("Timeout waiting for data")
                continue

            # Check header
            if len(data) < 12 or data[:4] != FRAME_HEADER:
                continue

            # Parse header
            packet_type = struct.unpack('<I', data[4:8])[0]
            packet_size = struct.unpack('<I', data[8:12])[0]

            packet_counts[packet_type] = packet_counts.get(packet_type, 0) + 1

            # Only analyze IMU packets (type 104)
            if packet_type == LIDAR_IMU_DATA_PACKET_TYPE:
                imu_packets_analyzed += 1
                print(f"\n{'='*60}")
                print(f"IMU PACKET #{imu_packets_analyzed}")
                print(f"{'='*60}")
                print(f"Total packet size: {len(data)} bytes")
                print(f"Header packet_type: {packet_type}")
                print(f"Header packet_size: {packet_size}")

                # Extract payload (skip 12-byte header)
                # Also need to skip tail (last 12 bytes: crc32[4] + msg_type_check[4] + reserve[2] + tail[2])
                tail_offset = len(data) - 12
                payload = data[12:tail_offset]

                print(f"Payload size: {len(payload)} bytes")

                parse_imu_multiple_offsets(payload)

                if imu_packets_analyzed >= max_imu_packets:
                    print(f"\n\nAnalyzed {max_imu_packets} IMU packets. Stopping.")
                    break

            # Status every 100 packets
            total_packets = sum(packet_counts.values())
            if total_packets % 100 == 0:
                elapsed = int(time.time() - start_time)
                print(f"\r[{elapsed}s] Packets - 3D:{packet_counts.get(102,0)} 2D:{packet_counts.get(103,0)} IMU:{packet_counts.get(104,0)}", end='')

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        sock.close()

    print("\n\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"Packet counts:")
    for ptype, count in sorted(packet_counts.items()):
        name = {102: "3D Point", 103: "2D Point", 104: "IMU"}.get(ptype, f"Type {ptype}")
        print(f"  {name}: {count}")

    if packet_counts.get(104, 0) == 0:
        print("\n*** WARNING: No IMU packets received! ***")
        print("The LiDAR may not be sending IMU data, or")
        print("the packet type 104 detection is incorrect.")


if __name__ == "__main__":
    main()
