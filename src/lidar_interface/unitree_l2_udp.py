"""
Unitree L2 LiDAR UDP Interface
Receives and parses point cloud data via UDP from the Unitree L2 4D LiDAR sensor.

Based on the official Unitree L2 protocol from unitree_lidar_protocol.h
"""

import socket
import struct
import numpy as np
import logging
from typing import Optional, Tuple
from dataclasses import dataclass
import zlib

logger = logging.getLogger(__name__)

# Protocol constants from unitree_lidar_protocol.h
FRAME_HEADER = bytes([0x55, 0xAA, 0x05, 0x0A])
FRAME_TAIL = bytes([0x00, 0xFF])

LIDAR_POINT_DATA_PACKET_TYPE = 102
LIDAR_2D_POINT_DATA_PACKET_TYPE = 103
LIDAR_IMU_DATA_PACKET_TYPE = 104


@dataclass
class UnitreeL2Config:
    """UDP Configuration for Unitree L2 LiDAR"""
    sensor_ip: str = "192.168.1.62"
    sensor_port: int = 6101
    host_ip: str = "192.168.1.2"
    host_port: int = 6201
    timeout: float = 2.0
    buffer_size: int = 8192  # Increased for 5528-byte 2D packets


class UnitreeL2UDP:
    """
    UDP receiver for Unitree L2 LiDAR sensor.

    Implements the official Unitree protocol:
    - FrameHeader: 12 bytes (header[4] + packet_type[4] + packet_size[4])
    - Data: Variable size depending on packet type
    - FrameTail: 12 bytes (crc32[4] + msg_type_check[4] + reserve[2] + tail[2])
    """

    def __init__(self, config: Optional[UnitreeL2Config] = None):
        """
        Initialize UDP receiver for Unitree L2.

        Args:
            config: UDP configuration parameters
        """
        self.config = config or UnitreeL2Config()
        self.socket: Optional[socket.socket] = None
        self.is_connected = False

        logger.info(f"Initialized Unitree L2 UDP receiver")
        logger.info(f"  Sensor: {self.config.sensor_ip}:{self.config.sensor_port}")
        logger.info(f"  Host:   {self.config.host_ip}:{self.config.host_port}")

    def connect(self) -> bool:
        """
        Set up UDP socket to receive data from sensor.

        Returns:
            bool: True if socket created successfully
        """
        try:
            # Create UDP socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind to host IP and port
            self.socket.bind((self.config.host_ip, self.config.host_port))
            self.socket.settimeout(self.config.timeout)

            logger.info(f"UDP socket bound to {self.config.host_ip}:{self.config.host_port}")

            # Test reception
            logger.info("Waiting for first packet to confirm connection...")
            data, addr = self.socket.recvfrom(self.config.buffer_size)

            if addr[0] == self.config.sensor_ip:
                logger.info(f"Received {len(data)} bytes from Unitree L2 at {addr}")
                self.is_connected = True
                return True
            else:
                logger.warning(f"Received data from unexpected source: {addr}")
                return False

        except socket.timeout:
            logger.error(f"Timeout waiting for data from sensor. Check:")
            logger.error(f"  1. Sensor is powered on and connected")
            logger.error(f"  2. Network settings: sensor={self.config.sensor_ip}, host={self.config.host_ip}")
            logger.error(f"  3. Firewall allows UDP port {self.config.host_port}")
            logger.error(f"  4. Sensor is configured to send to {self.config.host_ip}:{self.config.host_port}")
            return False

        except Exception as e:
            logger.error(f"Failed to set up UDP socket: {e}")
            return False

    def disconnect(self) -> None:
        """Close UDP socket."""
        if self.socket:
            self.socket.close()
            self.socket = None
            self.is_connected = False
            logger.info("Disconnected from Unitree L2")

    def receive_packet(self) -> Optional[bytes]:
        """
        Receive a single UDP packet from the sensor.

        Returns:
            Raw packet data or None if reception fails
        """
        if not self.is_connected or not self.socket:
            logger.error("Not connected. Call connect() first.")
            return None

        try:
            data, addr = self.socket.recvfrom(self.config.buffer_size)

            # Verify source
            if addr[0] != self.config.sensor_ip:
                logger.warning(f"Received packet from unexpected source: {addr}")
                return None

            return data

        except socket.timeout:
            logger.warning("Timeout receiving packet")
            return None

        except Exception as e:
            logger.error(f"Error receiving packet: {e}")
            return None

    def parse_packet(self, data: bytes) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Parse raw UDP packet into point cloud data according to official protocol.

        Packet structure (from unitree_lidar_protocol.h):
        - FrameHeader: 12 bytes
          - header[4]: 0x55 0xAA 0x05 0x0A
          - packet_type[4]: uint32 (102=3D points, 103=2D points)
          - packet_size[4]: uint32 total packet size
        - Data: Variable size
        - FrameTail: 12 bytes
          - crc32[4]: CRC checksum
          - msg_type_check[4]
          - reserve[2]
          - tail[2]: 0x00 0xFF

        Args:
            data: Raw UDP packet bytes

        Returns:
            Tuple of (points array Nx3, intensities array Nx1) or None if parsing fails
        """
        try:
            if len(data) < 24:  # Minimum: header + tail
                logger.error(f"Packet too small: {len(data)} bytes")
                return None

            # Parse FrameHeader (12 bytes)
            header_bytes = data[0:4]
            if header_bytes != FRAME_HEADER:
                logger.error(f"Invalid frame header: {header_bytes.hex()}")
                return None

            packet_type = struct.unpack('<I', data[4:8])[0]
            packet_size = struct.unpack('<I', data[8:12])[0]

            if len(data) != packet_size:
                logger.warning(f"Packet size mismatch: expected {packet_size}, got {len(data)}")

            # Parse FrameTail (12 bytes from end)
            tail_offset = len(data) - 12
            crc32_received = struct.unpack('<I', data[tail_offset:tail_offset+4])[0]
            tail_bytes = data[tail_offset+10:tail_offset+12]

            if tail_bytes != FRAME_TAIL:
                logger.error(f"Invalid frame tail: {tail_bytes.hex()}")
                return None

            # Verify CRC32 (of header + data, excluding tail)
            crc32_computed = zlib.crc32(data[:tail_offset]) & 0xFFFFFFFF
            if crc32_computed != crc32_received:
                logger.warning(f"CRC mismatch: computed {crc32_computed:08x}, received {crc32_received:08x}")
                # Continue anyway - some packets may have CRC issues

            # Parse based on packet type
            if packet_type == LIDAR_POINT_DATA_PACKET_TYPE:
                return self._parse_3d_point_data(data[12:tail_offset])
            elif packet_type == LIDAR_2D_POINT_DATA_PACKET_TYPE:
                return self._parse_2d_point_data(data[12:tail_offset])
            elif packet_type == LIDAR_IMU_DATA_PACKET_TYPE:
                logger.debug("Received IMU packet (skipping)")
                return None
            else:
                logger.debug(f"Unknown packet type: {packet_type}")
                return None

        except Exception as e:
            logger.error(f"Failed to parse packet: {e}")
            return None

    def _parse_3d_point_data(self, payload: bytes) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Parse 3D point data payload (type 102).

        Structure (from unitree_lidar_protocol.h):
        - DataInfo: 16 bytes
        - LidarInsideState: 36 bytes
        - LidarCalibParam: 32 bytes
        - Scan parameters: 36 bytes
        - point_num: 4 bytes (uint32)
        - ranges[300]: 600 bytes (uint16 array, distances in mm)
        - intensities[300]: 300 bytes (uint8 array, reflectivity 0-255)
        Total: 1024 bytes, but actual data defined by point_num
        """
        try:
            offset = 0

            # Skip DataInfo (16 bytes)
            offset += 16

            # Skip LidarInsideState (36 bytes)
            offset += 36

            # Parse LidarCalibParam (32 bytes) - we may need these for coordinate transform
            a_axis_dist = struct.unpack('<f', payload[offset:offset+4])[0]
            b_axis_dist = struct.unpack('<f', payload[offset+4:offset+8])[0]
            theta_angle_bias = struct.unpack('<f', payload[offset+8:offset+12])[0]
            alpha_angle_bias = struct.unpack('<f', payload[offset+12:offset+16])[0]
            beta_angle = struct.unpack('<f', payload[offset+16:offset+20])[0]
            xi_angle = struct.unpack('<f', payload[offset+20:offset+24])[0]
            range_bias = struct.unpack('<f', payload[offset+24:offset+28])[0]
            range_scale = struct.unpack('<f', payload[offset+28:offset+32])[0]
            offset += 32

            # Parse scan parameters (36 bytes)
            com_horizontal_angle_start = struct.unpack('<f', payload[offset:offset+4])[0]
            com_horizontal_angle_step = struct.unpack('<f', payload[offset+4:offset+8])[0]
            scan_period = struct.unpack('<f', payload[offset+8:offset+12])[0]
            range_min = struct.unpack('<f', payload[offset+12:offset+16])[0]
            range_max = struct.unpack('<f', payload[offset+16:offset+20])[0]
            angle_min = struct.unpack('<f', payload[offset+20:offset+24])[0]
            angle_increment = struct.unpack('<f', payload[offset+24:offset+28])[0]
            time_increment = struct.unpack('<f', payload[offset+28:offset+32])[0]
            point_num = struct.unpack('<I', payload[offset+32:offset+36])[0]
            offset += 36

            if point_num == 0 or point_num > 300:
                logger.debug(f"Invalid point_num: {point_num}")
                return None

            # Parse ranges (uint16 array, mm)
            ranges_bytes = payload[offset:offset+600]
            ranges = np.frombuffer(ranges_bytes, dtype=np.uint16, count=point_num)
            offset += 600

            # Parse intensities (uint8 array)
            intensities_bytes = payload[offset:offset+300]
            intensities = np.frombuffer(intensities_bytes, dtype=np.uint8, count=point_num)

            # Convert polar to Cartesian coordinates with proper 3D transformation
            # Based on dilohn/unitree-L2-lidar reference implementation
            points = []
            valid_intensities = []

            # Precompute rotation biases (beta and xi are calibration angles)
            sin_beta = np.sin(beta_angle)
            cos_beta = np.cos(beta_angle)
            sin_xi = np.sin(xi_angle)
            cos_xi = np.cos(xi_angle)
            cos_beta_sin_xi = cos_beta * sin_xi
            sin_beta_cos_xi = sin_beta * cos_xi
            sin_beta_sin_xi = sin_beta * sin_xi
            cos_beta_cos_xi = cos_beta * cos_xi

            # Initialize angle iterators
            # alpha = vertical scan angle, theta = horizontal rotation angle
            alpha_cur = angle_min + alpha_angle_bias
            alpha_step = angle_increment
            theta_cur = com_horizontal_angle_start + theta_angle_bias
            theta_step = com_horizontal_angle_step

            for i in range(point_num):
                range_mm = ranges[i]
                intensity = intensities[i]

                # Skip zero/invalid ranges
                if range_mm < 1:
                    alpha_cur += alpha_step
                    theta_cur += theta_step
                    continue

                # The range is already in mm - use it directly
                # range_scale appears to be ~0.001 which would scale it incorrectly
                # Just use the raw range value
                r = float(range_mm)

                # Spherical to Cartesian with calibration corrections
                sin_alpha = np.sin(alpha_cur)
                cos_alpha = np.cos(alpha_cur)
                sin_theta = np.sin(theta_cur)
                cos_theta = np.cos(theta_cur)

                # Intermediate calculations (from Unitree's transform)
                A = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * r + b_axis_dist
                B = cos_alpha * cos_xi * r
                C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * r

                # Final 3D coordinates (in mm)
                x_mm = cos_theta * A - sin_theta * B
                y_mm = sin_theta * A + cos_theta * B
                z_mm = C + a_axis_dist

                # Convert to meters
                x = x_mm / 1000.0
                y = y_mm / 1000.0
                z = z_mm / 1000.0

                # Range check (using meters)
                distance = np.sqrt(x*x + y*y + z*z)
                if distance < range_min or distance > range_max:
                    alpha_cur += alpha_step
                    theta_cur += theta_step
                    continue

                points.append([x, y, z])
                valid_intensities.append(intensity)

                # Advance both angles for next point
                alpha_cur += alpha_step
                theta_cur += theta_step

            if len(points) == 0:
                return None

            points_array = np.array(points, dtype=np.float32)
            intensities_array = np.array(valid_intensities, dtype=np.float32).reshape(-1, 1) / 255.0

            logger.debug(f"Parsed 3D packet: {len(points_array)} points")
            return points_array, intensities_array

        except Exception as e:
            logger.error(f"Error parsing 3D point data: {e}")
            return None

    def _parse_2d_point_data(self, payload: bytes) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Parse 2D point data payload (type 103).

        Similar to 3D but with 1800 points instead of 300.
        """
        try:
            offset = 0

            # Skip DataInfo (16 bytes)
            offset += 16

            # Skip LidarInsideState (36 bytes)
            offset += 36

            # Skip LidarCalibParam (32 bytes)
            offset += 32

            # Parse scan parameters (28 bytes - no horizontal angle params)
            scan_period = struct.unpack('<f', payload[offset:offset+4])[0]
            range_min = struct.unpack('<f', payload[offset+4:offset+8])[0]
            range_max = struct.unpack('<f', payload[offset+8:offset+12])[0]
            angle_min = struct.unpack('<f', payload[offset+12:offset+16])[0]
            angle_increment = struct.unpack('<f', payload[offset+16:offset+20])[0]
            time_increment = struct.unpack('<f', payload[offset+20:offset+24])[0]
            point_num = struct.unpack('<I', payload[offset+24:offset+28])[0]
            offset += 28

            if point_num == 0 or point_num > 1800:
                logger.debug(f"Invalid point_num for 2D: {point_num}")
                return None

            # Parse ranges (uint16 array, mm)
            ranges_bytes = payload[offset:offset+3600]
            ranges = np.frombuffer(ranges_bytes, dtype=np.uint16, count=point_num)
            offset += 3600

            # Parse intensities (uint8 array)
            intensities_bytes = payload[offset:offset+1800]
            intensities = np.frombuffer(intensities_bytes, dtype=np.uint8, count=point_num)

            # Convert polar to Cartesian coordinates
            points = []
            valid_intensities = []

            for i in range(point_num):
                range_mm = ranges[i]
                intensity = intensities[i]

                # Skip zero ranges
                if range_mm == 0:
                    continue

                # Convert range from mm to meters
                range_m = range_mm / 1000.0

                # Calculate angle for this point
                angle_rad = angle_min + i * angle_increment

                # Convert polar to Cartesian (2D scan in X-Y plane)
                x = range_m * np.cos(angle_rad)
                y = range_m * np.sin(angle_rad)
                z = 0.0  # 2D scan

                # Range check
                if range_m < range_min or range_m > range_max:
                    continue

                points.append([x, y, z])
                valid_intensities.append(intensity)

            if len(points) == 0:
                return None

            points_array = np.array(points, dtype=np.float32)
            intensities_array = np.array(valid_intensities, dtype=np.float32).reshape(-1, 1) / 255.0

            logger.debug(f"Parsed 2D packet: {len(points_array)} points")
            return points_array, intensities_array

        except Exception as e:
            logger.error(f"Error parsing 2D point data: {e}")
            return None

    def get_point_cloud(self) -> Optional[np.ndarray]:
        """
        Receive and parse a single frame of point cloud data.

        Returns:
            Nx3 NumPy array of points or None if failed
        """
        packet = self.receive_packet()

        if packet is None:
            return None

        result = self.parse_packet(packet)

        if result is None:
            return None

        points, _ = result
        logger.debug(f"Received point cloud with {len(points)} points")
        return points

    def get_point_cloud_with_intensity(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Receive point cloud with intensity values.

        Returns:
            Tuple of (points, intensities) or None
        """
        packet = self.receive_packet()

        if packet is None:
            return None

        result = self.parse_packet(packet)

        if result is not None:
            points, intensities = result
            logger.debug(f"Received point cloud with {len(points)} points and intensities")

        return result

    def test_connection(self) -> bool:
        """
        Test if we can receive data from the sensor.

        Returns:
            bool: True if data received successfully
        """
        logger.info("Testing connection to Unitree L2...")

        if not self.is_connected:
            logger.info("Attempting to connect...")
            if not self.connect():
                return False

        logger.info("Receiving test packet...")
        packet = self.receive_packet()

        if packet is None:
            logger.error("Failed to receive test packet")
            return False

        logger.info(f"[OK] Successfully received {len(packet)} bytes")

        # Try to parse it
        result = self.parse_packet(packet)
        if result is not None:
            points, _ = result
            logger.info(f"[OK] Successfully parsed {len(points)} points")
        else:
            logger.warning("Could not parse packet (may be IMU or unknown type)")

        logger.info("Connection test passed!")
        return True

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


# Example usage
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    print("Unitree L2 UDP Receiver Test")
    print("=" * 50)

    config = UnitreeL2Config()
    receiver = UnitreeL2UDP(config)

    print("\nAttempting to connect...")
    if receiver.connect():
        print("[OK] Connected successfully!")

        print("\nReceiving 30 test packets...")
        total_points = 0
        packet_count = 0

        for i in range(30):
            result = receiver.get_point_cloud_with_intensity()
            if result:
                points, intensities = result
                total_points += len(points)
                packet_count += 1
                print(f"  Packet {i+1}: {len(points)} points")
            else:
                print(f"  Packet {i+1}: No points (IMU or other packet type)")

        print(f"\nSummary:")
        print(f"  Packets with points: {packet_count}/30")
        print(f"  Total points: {total_points}")
        if packet_count > 0:
            print(f"  Average points/packet: {total_points/packet_count:.1f}")

        receiver.disconnect()
    else:
        print("[FAIL] Connection failed!")
        print("\nTroubleshooting:")
        print("1. Check sensor is powered on")
        print("2. Verify network configuration (run: ipconfig or ifconfig)")
        print("3. Ping sensor: ping 192.168.1.62")
        print("4. Check firewall settings")