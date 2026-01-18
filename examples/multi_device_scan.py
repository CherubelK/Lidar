"""
Multi-device collaborative LiDAR scanning example

This demonstrates how to set up multiple LiDAR devices in a mesh network
to create a single large-scale map through collaborative scanning.

Usage:
    # On master device (aggregates all maps)
    python multi_device_scan.py --role master --device-id LIDAR_MASTER

    # On each node device
    python multi_device_scan.py --role node --device-id LIDAR_001 --master <master_bt_address>
"""

import sys
from pathlib import Path
import argparse
import time
import numpy as np
from datetime import datetime

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface.unitree_l2_udp import UnitreeL2UDP
from src.data_processing.kiss_icp_slam import KissICPOdometry
from src.networking.mesh_network import BluetoothMeshNetwork, WiFiMeshNetwork, DeviceRole
from src.networking.encryption import EncryptionManager
from src.networking.map_merger import MapMerger, DeviceMap, compress_pointcloud, decompress_pointcloud

try:
    from src.positioning.gps_rtk import GPSRTK
    GPS_AVAILABLE = True
except ImportError:
    GPS_AVAILABLE = False
    print("Warning: GPS module not available")


class MultiDeviceScanner:
    """Multi-device collaborative scanner"""

    def __init__(self, device_id: str, role: DeviceRole,
                 use_wifi: bool = False,
                 use_gps: bool = False,
                 encryption_key: str = None):
        """
        Initialize multi-device scanner

        Args:
            device_id: Unique device identifier
            role: MASTER or NODE
            use_wifi: Use WiFi instead of Bluetooth
            use_gps: Enable GPS positioning
            encryption_key: Path to shared encryption key (optional)
        """
        self.device_id = device_id
        self.role = role
        self.use_gps = use_gps and GPS_AVAILABLE

        # Initialize encryption
        self.encryption_mgr = EncryptionManager(device_id, encryption_key)

        # Initialize network
        if use_wifi:
            print(f"Using WiFi mesh network")
            self.network = WiFiMeshNetwork(device_id, role, self.encryption_mgr)
        else:
            print(f"Using Bluetooth mesh network")
            self.network = BluetoothMeshNetwork(device_id, role, self.encryption_mgr)

        # Set up callbacks
        if role == DeviceRole.MASTER:
            self.network.on_data_received = self._on_map_received
            self.network.on_device_connected = self._on_device_connected
            self.network.on_device_disconnected = self._on_device_disconnected

            # Initialize map merger
            merge_strategy = 'hybrid' if use_gps else 'icp'
            self.map_merger = MapMerger(strategy=merge_strategy, voxel_size=0.05)

        # LiDAR interface
        self.lidar: Optional[UnitreeL2UDP] = None

        # GPS interface
        self.gps: Optional[GPSRTK] = None
        if self.use_gps:
            try:
                self.gps = GPSRTK()
                print("GPS initialized")
            except Exception as e:
                print(f"GPS initialization failed: {e}")
                self.gps = None

        # KISS-ICP odometry
        self.odometry = KissICPOdometry()

        # Scanning state
        self.is_scanning = False
        self.local_points = []
        self.local_intensities = []
        self.gps_origin = None

    def _on_device_connected(self, device_id: str):
        """Callback when device connects (master only)"""
        print(f"\n✓ Device connected: {device_id}")
        print(f"  Total devices: {len(self.network.get_connected_devices())}")

    def _on_device_disconnected(self, device_id: str):
        """Callback when device disconnects (master only)"""
        print(f"\n✗ Device disconnected: {device_id}")

    def _on_map_received(self, sender_id: str, data: bytes):
        """Callback when map received from node (master only)"""
        print(f"\nReceived map from {sender_id} ({len(data):,} bytes)")

        try:
            # Decompress point cloud
            import json
            import lz4.frame

            # Extract metadata and points
            # Format: [metadata_len(4)] [metadata_json] [compressed_points]
            metadata_len = int.from_bytes(data[:4], 'big')
            metadata_json = data[4:4+metadata_len].decode('utf-8')
            compressed_points = data[4+metadata_len:]

            metadata = json.loads(metadata_json)

            # Decompress
            points = decompress_pointcloud(compressed_points)

            print(f"  Decompressed: {len(points):,} points")

            # Create device map
            device_map = DeviceMap(
                device_id=sender_id,
                points=points,
                intensities=None,  # Not transmitted to save bandwidth
                gps_origin=tuple(metadata['gps_origin']) if metadata.get('has_gps') else None,
                has_gps=metadata.get('has_gps', False)
            )

            # Merge into master map
            self.map_merger.add_device_map(device_map)

            print(f"  Master map now: {self.map_merger.get_stats()['points']:,} points")

        except Exception as e:
            print(f"  ERROR: Failed to process map from {sender_id}: {e}")

    def start_network(self, master_address: str = None, master_ip: str = None):
        """
        Start mesh network

        Args:
            master_address: Bluetooth MAC address of master (for nodes)
            master_ip: IP address of master (for WiFi nodes)
        """
        print("\n=== Starting Mesh Network ===")

        if self.role == DeviceRole.MASTER:
            # Start as master
            if isinstance(self.network, WiFiMeshNetwork):
                self.network.start()  # WiFi binds to all interfaces
            else:
                self.network.start()  # Bluetooth starts advertising

            print(f"\n✓ Master {self.device_id} is ready")
            print(f"  Waiting for nodes to connect...")

            if isinstance(self.network, WiFiMeshNetwork):
                import socket
                hostname = socket.gethostname()
                ip = socket.gethostbyname(hostname)
                print(f"  WiFi IP: {ip}")
            else:
                # Show Bluetooth address
                import bluetooth
                bt_addr = bluetooth.read_local_bdaddr()[0]
                print(f"  Bluetooth Address: {bt_addr}")

        else:
            # Connect to master
            if isinstance(self.network, WiFiMeshNetwork):
                if not master_ip:
                    raise ValueError("Must provide --master-ip for WiFi nodes")
                success = self.network.connect_to_master(master_ip)
            else:
                if not master_address:
                    raise ValueError("Must provide --master for Bluetooth nodes")
                success = self.network.connect_to_master(master_address)

            if success:
                print(f"\n✓ Node {self.device_id} connected to master")
            else:
                print(f"\n✗ Failed to connect to master")
                return False

        return True

    def start_scan(self, duration: int = 60):
        """
        Start LiDAR scanning

        Args:
            duration: Scan duration in seconds
        """
        print(f"\n=== Starting LiDAR Scan ({duration}s) ===")

        # Initialize LiDAR
        try:
            self.lidar = UnitreeL2UDP()
            print("✓ LiDAR connected")
        except Exception as e:
            print(f"✗ LiDAR connection failed: {e}")
            return

        # Get GPS origin (if available)
        if self.gps:
            print("Waiting for GPS fix...")
            for _ in range(10):
                pos = self.gps.get_position()
                if pos:
                    self.gps_origin = (pos['latitude'], pos['longitude'], pos['altitude'])
                    print(f"✓ GPS fix acquired: {self.gps_origin}")
                    break
                time.sleep(1)
            else:
                print("⚠ No GPS fix, proceeding without GPS")

        # Scan
        self.is_scanning = True
        start_time = time.time()
        scan_count = 0

        print("\nScanning... (move SLOWLY for best results)")

        while time.time() - start_time < duration and self.is_scanning:
            # Get LiDAR scan
            points, intensity = self.lidar.get_point_cloud()

            if points is None or len(points) == 0:
                continue

            # Process with KISS-ICP
            transformed_points, pose, success = self.odometry.process_scan(points)

            if success:
                self.local_points.append(transformed_points)
                if intensity is not None:
                    self.local_intensities.append(intensity)

                scan_count += 1

                if scan_count % 10 == 0:
                    elapsed = int(time.time() - start_time)
                    print(f"  {elapsed}s: {scan_count} scans, "
                          f"{sum(len(p) for p in self.local_points):,} points")

            time.sleep(0.2)  # 5 Hz

        print(f"\n✓ Scan complete: {scan_count} scans, "
              f"{sum(len(p) for p in self.local_points):,} total points")

        # Cleanup
        self.lidar.close()
        if self.gps:
            self.gps.close()

    def send_map_to_master(self):
        """Send local map to master (node only)"""
        if self.role != DeviceRole.NODE:
            print("Only nodes can send maps to master")
            return

        if not self.local_points:
            print("No points to send")
            return

        print("\n=== Sending Map to Master ===")

        # Merge local scans
        merged_points = np.vstack(self.local_points)
        print(f"Local map: {len(merged_points):,} points")

        # Compress
        print("Compressing...")
        compressed = compress_pointcloud(merged_points, voxel_size=0.05)

        # Create metadata
        import json
        metadata = {
            'device_id': self.device_id,
            'points': len(merged_points),
            'has_gps': self.gps_origin is not None,
            'gps_origin': self.gps_origin if self.gps_origin else [0, 0, 0],
            'timestamp': datetime.now().isoformat()
        }
        metadata_json = json.dumps(metadata).encode('utf-8')

        # Pack: [metadata_len] [metadata] [compressed_points]
        packed = len(metadata_json).to_bytes(4, 'big') + metadata_json + compressed

        print(f"Compressed: {len(merged_points):,} points → {len(packed):,} bytes "
              f"({len(packed)/len(merged_points)/12*100:.1f}% of original)")

        # Send
        print("Sending to master...")
        success = self.network.send_data(packed, 'master')

        if success:
            print("✓ Map sent successfully")
        else:
            print("✗ Failed to send map")

    def save_merged_map(self, output_dir: str):
        """Save merged map (master only)"""
        if self.role != DeviceRole.MASTER:
            print("Only master can save merged map")
            return

        print("\n=== Saving Merged Map ===")

        # Add master's own map
        if self.local_points:
            print("Adding master's own map...")
            merged_points = np.vstack(self.local_points)

            master_map = DeviceMap(
                device_id=self.device_id,
                points=merged_points,
                gps_origin=self.gps_origin,
                has_gps=self.gps_origin is not None
            )

            self.map_merger.add_device_map(master_map)

        # Downsample
        self.map_merger.downsample_merged(voxel_size=0.02)

        # Remove outliers
        self.map_merger.remove_outliers()

        # Save
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"{output_dir}/multi_device_{timestamp}"

        self.map_merger.save_merged_map(output_path, prefix="merged")

        print(f"\n✓ Merged map saved to {output_path}")
        print(f"  Final map: {self.map_merger.get_stats()['points']:,} points")

    def stop(self):
        """Stop scanner and network"""
        self.is_scanning = False

        if self.lidar:
            self.lidar.close()

        if self.gps:
            self.gps.close()

        self.network.stop()


def main():
    parser = argparse.ArgumentParser(description="Multi-device LiDAR scanning")

    parser.add_argument('--role', type=str, required=True,
                       choices=['master', 'node'],
                       help='Device role')

    parser.add_argument('--device-id', type=str, required=True,
                       help='Unique device identifier (e.g., LIDAR_001)')

    parser.add_argument('--master', type=str,
                       help='Bluetooth MAC address of master device (for nodes)')

    parser.add_argument('--master-ip', type=str,
                       help='IP address of master device (for WiFi nodes)')

    parser.add_argument('--wifi', action='store_true',
                       help='Use WiFi instead of Bluetooth')

    parser.add_argument('--gps', action='store_true',
                       help='Enable GPS positioning')

    parser.add_argument('--duration', type=int, default=60,
                       help='Scan duration in seconds (default: 60)')

    parser.add_argument('--output-dir', type=str, default='data/multi_device',
                       help='Output directory for merged map')

    parser.add_argument('--encryption-key', type=str,
                       help='Path to shared encryption key file')

    args = parser.parse_args()

    # Convert role
    role = DeviceRole.MASTER if args.role == 'master' else DeviceRole.NODE

    # Create scanner
    scanner = MultiDeviceScanner(
        device_id=args.device_id,
        role=role,
        use_wifi=args.wifi,
        use_gps=args.gps,
        encryption_key=args.encryption_key
    )

    try:
        # Start network
        if role == DeviceRole.MASTER:
            scanner.start_network()

            print("\n=== Master Device ===")
            print("Waiting for nodes to connect and send maps...")
            print("Press Ctrl+C when all scans are complete")

            # Wait for user interrupt
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n\nSaving merged map...")

        else:
            # Node mode
            if not scanner.start_network(
                master_address=args.master,
                master_ip=args.master_ip
            ):
                return

            print("\n=== Node Device ===")
            input("Press Enter to start scanning...")

            # Scan
            scanner.start_scan(duration=args.duration)

            # Send to master
            scanner.send_map_to_master()

            print("\n✓ Done. You can now disconnect.")

        # Save (master only)
        if role == DeviceRole.MASTER:
            scanner.save_merged_map(args.output_dir)

    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        scanner.stop()


if __name__ == "__main__":
    main()