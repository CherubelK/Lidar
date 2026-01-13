"""
Unitree L2 LiDAR UDP Interface
Receives and parses point cloud data via UDP from the Unitree L2 4D LiDAR sensor.

Based on the Unitree L2 protocol. Requires sensor configuration:
- Packet Type: 3D point packets
- IMU: Disabled (or handle separately)
- Mode: Normal (not negative angle)
"""

import socket
import struct
import numpy as np
import logging
from typing import Optional, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class UnitreeL2Config:
    """UDP Configuration for Unitree L2 LiDAR"""
    sensor_ip: str = "192.168.1.62"
    sensor_port: int = 6101
    host_ip: str = "192.168.1.2"
    host_port: int = 6201
    timeout: float = 2.0
    buffer_size: int = 4096


class UnitreeL2UDP:
    """
    UDP receiver for Unitree L2 LiDAR sensor.

    Note: This is a basic implementation template. You'll need to:
    1. Reference the official Unitree protocol specification
    2. Implement proper packet parsing based on the protocol
    3. Test with actual hardware

    For a working Python implementation, see:
    https://github.com/dilohn/unitree-L2-lidar
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

    def parse_packet(self, data: bytes) -> Optional[np.ndarray]:
        """
        Parse raw UDP packet into point cloud data.

        This is a PLACEHOLDER implementation. You need to:
        1. Study the Unitree L2 packet format from official documentation
        2. Implement proper binary parsing using struct.unpack()
        3. Extract x, y, z coordinates and other fields
        4. Return as Nx3 NumPy array

        For reference implementation, see:
        https://github.com/dilohn/unitree-L2-lidar/blob/main/decode_lidar_3d.py

        Args:
            data: Raw UDP packet bytes

        Returns:
            Nx3 NumPy array of points (x, y, z) or None if parsing fails
        """
        try:
            if len(data) < 20:
                logger.error(f"Packet too small: {len(data)} bytes")
                return None

            # Unitree L2 packet structure (empirically determined):
            # Header: ~20 bytes
            # Points: x, y, z as float32 (12 bytes per point)

            header_size = 20
            point_size = 12  # x, y, z as float32

            payload_size = len(data) - header_size

            # Try different point sizes to find the right one
            for ps in [12, 16, 20, 24]:
                if payload_size % ps == 0:
                    point_size = ps
                    break

            num_points = payload_size // point_size

            if num_points == 0:
                return None

            points = []
            offset = header_size

            # Parse points
            for i in range(num_points):
                if offset + 12 > len(data):
                    break

                try:
                    # Extract x, y, z as float32 (little-endian)
                    # NOTE: Based on diagnostic analysis, swapping Y and Z to correct coordinate system
                    # Unitree L2 coordinate system: +X (opposite cable outlet), +Y (90° CCW from +X), +Z (perpendicular up)
                    x_raw = struct.unpack('<f', data[offset:offset+4])[0]
                    y_raw = struct.unpack('<f', data[offset+4:offset+8])[0]
                    z_raw = struct.unpack('<f', data[offset+8:offset+12])[0]

                    # Swap Y and Z to match expected coordinate system
                    x = x_raw
                    y = z_raw  # Z from packet becomes Y (depth/forward)
                    z = y_raw  # Y from packet becomes Z (up/down)

                    # Skip invalid points
                    if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                        offset += point_size
                        continue

                    # Range check (0.1m to 50m)
                    distance = np.sqrt(x*x + y*y + z*z)
                    if distance > 0.1 and distance < 50.0:
                        points.append([x, y, z])

                    offset += point_size

                except Exception as e:
                    logger.debug(f"Error parsing point {i}: {e}")
                    offset += point_size
                    continue

            if len(points) == 0:
                return None

            result = np.array(points, dtype=np.float32)
            logger.debug(f"Parsed {len(result)} valid points from packet")
            return result

        except Exception as e:
            logger.error(f"Failed to parse packet: {e}")
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

        points = self.parse_packet(packet)

        if points is not None:
            logger.debug(f"Received point cloud with {len(points)} points")

        return points

    def get_point_cloud_with_intensity(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Receive point cloud with intensity values.

        Note: Requires implementing intensity parsing in parse_packet().

        Returns:
            Tuple of (points, intensities) or None
        """
        # TODO: Implement intensity extraction in parse_packet()
        # Then return both points and intensities

        logger.warning("get_point_cloud_with_intensity() not fully implemented yet")
        points = self.get_point_cloud()

        if points is None:
            return None

        # Placeholder: return dummy intensities
        intensities = np.zeros((len(points), 1))
        return points, intensities

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

        logger.info(f"✓ Successfully received {len(packet)} bytes")
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
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    print("Unitree L2 UDP Receiver Test")
    print("=" * 50)

    config = UnitreeL2Config()
    receiver = UnitreeL2UDP(config)

    print("\nAttempting to connect...")
    if receiver.connect():
        print("✓ Connected successfully!")

        print("\nReceiving 5 test packets...")
        for i in range(5):
            packet = receiver.receive_packet()
            if packet:
                print(f"  Packet {i+1}: {len(packet)} bytes")
            else:
                print(f"  Packet {i+1}: Failed")

        receiver.disconnect()
    else:
        print("✗ Connection failed!")
        print("\nTroubleshooting:")
        print("1. Check sensor is powered on")
        print("2. Verify network configuration (run: ipconfig or ifconfig)")
        print("3. Ping sensor: ping 192.168.1.62")
        print("4. Check firewall settings")