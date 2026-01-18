"""
Bluetooth mesh networking for multi-device LiDAR collaboration
Supports both Bluetooth Classic (star topology) and BLE Mesh
"""

import bluetooth
import socket
import threading
import time
import json
from enum import Enum
from typing import List, Dict, Optional, Callable
from dataclasses import dataclass
from datetime import datetime

from .encryption import EncryptionManager, SecureMessage


class DeviceRole(Enum):
    """Device role in mesh network"""
    MASTER = "master"  # Aggregates data from all devices
    NODE = "node"      # Sends data to master
    RELAY = "relay"    # Forwards messages (for mesh extension)


@dataclass
class DeviceInfo:
    """Information about a mesh network device"""
    device_id: str
    bt_address: str
    role: DeviceRole
    last_seen: datetime
    message_counter: int = 0


class BluetoothMeshNetwork:
    """
    Bluetooth mesh network manager (Bluetooth Classic implementation)

    Uses RFCOMM protocol for reliable data transfer
    Supports up to 7 devices in star topology (1 master + 6 nodes)
    """

    def __init__(self, device_id: str, role: DeviceRole,
                 encryption_mgr: EncryptionManager,
                 port: int = 3):
        """
        Initialize Bluetooth mesh network

        Args:
            device_id: Unique device identifier
            role: Device role (MASTER or NODE)
            encryption_mgr: Encryption manager for secure communication
            port: RFCOMM port (default: 3)
        """
        self.device_id = device_id
        self.role = role
        self.encryption_mgr = encryption_mgr
        self.port = port

        # Network state
        self.devices: Dict[str, DeviceInfo] = {}
        self.connections: Dict[str, bluetooth.BluetoothSocket] = {}

        # Server socket (for receiving connections)
        self.server_socket: Optional[bluetooth.BluetoothSocket] = None
        self.running = False

        # Callbacks
        self.on_data_received: Optional[Callable] = None
        self.on_device_connected: Optional[Callable] = None
        self.on_device_disconnected: Optional[Callable] = None

        # Message counter for replay protection
        self.message_counter = 0

    def start(self):
        """Start network service (server for master, client for nodes)"""
        self.running = True

        if self.role == DeviceRole.MASTER:
            self._start_server()
        else:
            print(f"Node {self.device_id} ready. Use connect_to_master() to join network.")

    def _start_server(self):
        """Start server socket for accepting connections (master only)"""
        print(f"Starting Bluetooth server on port {self.port}...")

        self.server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.server_socket.bind(("", self.port))
        self.server_socket.listen(7)  # Max 7 connections

        # Make discoverable
        bluetooth.advertise_service(
            self.server_socket,
            "LidarMeshNetwork",
            service_id="1e0ca4ea-299d-4335-93eb-27fcfe7fa848",
            service_classes=["1e0ca4ea-299d-4335-93eb-27fcfe7fa848"],
            profiles=[bluetooth.SERIAL_PORT_PROFILE]
        )

        print(f"Master {self.device_id} listening for connections...")

        # Accept connections in background thread
        accept_thread = threading.Thread(target=self._accept_connections, daemon=True)
        accept_thread.start()

    def _accept_connections(self):
        """Accept incoming connections (runs in background thread)"""
        while self.running:
            try:
                client_sock, client_info = self.server_socket.accept()
                print(f"Accepted connection from {client_info}")

                # Handle client in separate thread
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_sock, client_info),
                    daemon=True
                )
                client_thread.start()

            except Exception as e:
                if self.running:
                    print(f"Error accepting connection: {e}")

    def _handle_client(self, sock: bluetooth.BluetoothSocket, client_info):
        """Handle connected client"""
        try:
            # Receive device ID
            device_id = sock.recv(64).decode('utf-8').strip()
            print(f"Device {device_id} connected from {client_info}")

            # Store connection
            self.connections[device_id] = sock
            self.devices[device_id] = DeviceInfo(
                device_id=device_id,
                bt_address=client_info[0],
                role=DeviceRole.NODE,
                last_seen=datetime.now()
            )

            # Callback
            if self.on_device_connected:
                self.on_device_connected(device_id)

            # Receive data
            while self.running:
                try:
                    # Receive message length (4 bytes)
                    length_bytes = self._recv_exact(sock, 4)
                    if not length_bytes:
                        break

                    message_length = int.from_bytes(length_bytes, 'big')

                    # Receive message data
                    message_data = self._recv_exact(sock, message_length)
                    if not message_data:
                        break

                    # Decrypt and verify
                    secure_msg = SecureMessage.unpack(message_data, self.encryption_mgr)

                    # Update device info
                    self.devices[device_id].last_seen = datetime.now()
                    self.devices[device_id].message_counter = secure_msg.counter

                    # Callback with decrypted data
                    if self.on_data_received:
                        self.on_data_received(device_id, secure_msg.data)

                except ValueError as e:
                    print(f"Message verification failed from {device_id}: {e}")
                    continue

        except Exception as e:
            print(f"Error handling client {device_id}: {e}")
        finally:
            print(f"Device {device_id} disconnected")
            if device_id in self.connections:
                del self.connections[device_id]
            if self.on_device_disconnected:
                self.on_device_disconnected(device_id)

    def _recv_exact(self, sock: bluetooth.BluetoothSocket, n: int) -> Optional[bytes]:
        """Receive exactly n bytes from socket"""
        data = b''
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def connect_to_master(self, master_address: str) -> bool:
        """
        Connect to master device (node only)

        Args:
            master_address: Bluetooth MAC address of master

        Returns:
            True if connected successfully
        """
        if self.role != DeviceRole.NODE:
            raise ValueError("Only nodes can connect to master")

        try:
            print(f"Connecting to master at {master_address}...")

            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            sock.connect((master_address, self.port))

            # Send our device ID
            sock.send(self.device_id.encode('utf-8').ljust(64))

            self.connections['master'] = sock
            self.devices['master'] = DeviceInfo(
                device_id='master',
                bt_address=master_address,
                role=DeviceRole.MASTER,
                last_seen=datetime.now()
            )

            print(f"Connected to master at {master_address}")
            return True

        except Exception as e:
            print(f"Failed to connect to master: {e}")
            return False

    def send_data(self, data: bytes, target: str = 'master') -> bool:
        """
        Send encrypted data to target device

        Args:
            data: Data to send
            target: Target device ID ('master' for nodes, specific device for master)

        Returns:
            True if sent successfully
        """
        if target not in self.connections:
            print(f"No connection to {target}")
            return False

        try:
            # Increment message counter
            self.message_counter += 1

            # Create secure message
            secure_msg = SecureMessage(self.device_id, data, self.message_counter)
            packed = secure_msg.pack(self.encryption_mgr)

            # Send message length + message
            sock = self.connections[target]
            sock.send(len(packed).to_bytes(4, 'big'))
            sock.send(packed)

            return True

        except Exception as e:
            print(f"Failed to send data to {target}: {e}")
            return False

    def broadcast(self, data: bytes) -> int:
        """
        Broadcast data to all connected devices (master only)

        Args:
            data: Data to broadcast

        Returns:
            Number of devices successfully sent to
        """
        if self.role != DeviceRole.MASTER:
            raise ValueError("Only master can broadcast")

        success_count = 0
        for device_id in list(self.connections.keys()):
            if self.send_data(data, device_id):
                success_count += 1

        return success_count

    def discover_devices(self, duration: int = 8) -> List[tuple]:
        """
        Discover nearby Bluetooth devices

        Args:
            duration: Discovery duration in seconds

        Returns:
            List of (address, name) tuples
        """
        print(f"Discovering Bluetooth devices for {duration} seconds...")
        devices = bluetooth.discover_devices(duration=duration, lookup_names=True)

        print(f"\nFound {len(devices)} devices:")
        for addr, name in devices:
            print(f"  {addr} - {name}")

        return devices

    def get_connected_devices(self) -> List[str]:
        """Get list of currently connected device IDs"""
        return list(self.connections.keys())

    def get_network_stats(self) -> dict:
        """Get network statistics"""
        return {
            'device_id': self.device_id,
            'role': self.role.value,
            'connected_devices': len(self.connections),
            'total_messages_sent': self.message_counter,
            'devices': {
                device_id: {
                    'last_seen': info.last_seen.isoformat(),
                    'messages_received': info.message_counter
                }
                for device_id, info in self.devices.items()
            }
        }

    def stop(self):
        """Stop network service and close all connections"""
        print("Stopping mesh network...")
        self.running = False

        # Close all connections
        for sock in self.connections.values():
            try:
                sock.close()
            except:
                pass
        self.connections.clear()

        # Close server socket
        if self.server_socket:
            try:
                bluetooth.stop_advertising(self.server_socket)
                self.server_socket.close()
            except:
                pass

        print("Mesh network stopped")


class WiFiMeshNetwork:
    """
    WiFi mesh network alternative (for larger deployments)

    Uses TCP sockets for higher throughput
    Supports 100+ devices with better range
    """

    def __init__(self, device_id: str, role: DeviceRole,
                 encryption_mgr: EncryptionManager,
                 port: int = 5555):
        """
        Initialize WiFi mesh network

        Args:
            device_id: Unique device identifier
            role: Device role (MASTER or NODE)
            encryption_mgr: Encryption manager
            port: TCP port (default: 5555)
        """
        self.device_id = device_id
        self.role = role
        self.encryption_mgr = encryption_mgr
        self.port = port

        # Similar structure to Bluetooth version but using TCP sockets
        self.devices: Dict[str, DeviceInfo] = {}
        self.connections: Dict[str, socket.socket] = {}
        self.server_socket: Optional[socket.socket] = None
        self.running = False

        self.on_data_received: Optional[Callable] = None
        self.on_device_connected: Optional[Callable] = None
        self.on_device_disconnected: Optional[Callable] = None

        self.message_counter = 0

    def start(self, bind_address: str = '0.0.0.0'):
        """Start WiFi mesh service"""
        self.running = True

        if self.role == DeviceRole.MASTER:
            self._start_server(bind_address)
        else:
            print(f"Node {self.device_id} ready. Use connect_to_master() to join network.")

    def _start_server(self, bind_address: str):
        """Start TCP server (master only)"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((bind_address, self.port))
        self.server_socket.listen(100)  # Support 100+ devices

        print(f"Master {self.device_id} listening on {bind_address}:{self.port}")

        # Accept connections in background
        accept_thread = threading.Thread(target=self._accept_connections, daemon=True)
        accept_thread.start()

    def _accept_connections(self):
        """Accept incoming TCP connections"""
        while self.running:
            try:
                client_sock, client_addr = self.server_socket.accept()
                print(f"Accepted connection from {client_addr}")

                # Handle client in separate thread
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_sock, client_addr),
                    daemon=True
                )
                client_thread.start()

            except Exception as e:
                if self.running:
                    print(f"Error accepting connection: {e}")

    def _handle_client(self, sock: socket.socket, client_addr):
        """Handle connected WiFi client (similar to Bluetooth version)"""
        try:
            # Receive device ID
            device_id = sock.recv(64).decode('utf-8').strip()
            print(f"Device {device_id} connected from {client_addr}")

            self.connections[device_id] = sock
            self.devices[device_id] = DeviceInfo(
                device_id=device_id,
                bt_address=client_addr[0],
                role=DeviceRole.NODE,
                last_seen=datetime.now()
            )

            if self.on_device_connected:
                self.on_device_connected(device_id)

            # Receive data
            while self.running:
                try:
                    # Receive message length
                    length_bytes = self._recv_exact(sock, 4)
                    if not length_bytes:
                        break

                    message_length = int.from_bytes(length_bytes, 'big')

                    # Receive message
                    message_data = self._recv_exact(sock, message_length)
                    if not message_data:
                        break

                    # Decrypt and verify
                    secure_msg = SecureMessage.unpack(message_data, self.encryption_mgr)

                    self.devices[device_id].last_seen = datetime.now()
                    self.devices[device_id].message_counter = secure_msg.counter

                    if self.on_data_received:
                        self.on_data_received(device_id, secure_msg.data)

                except ValueError as e:
                    print(f"Message verification failed from {device_id}: {e}")
                    continue

        except Exception as e:
            print(f"Error handling client {device_id}: {e}")
        finally:
            print(f"Device {device_id} disconnected")
            if device_id in self.connections:
                del self.connections[device_id]
            if self.on_device_disconnected:
                self.on_device_disconnected(device_id)

    def _recv_exact(self, sock: socket.socket, n: int) -> Optional[bytes]:
        """Receive exactly n bytes"""
        data = b''
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def connect_to_master(self, master_ip: str) -> bool:
        """Connect to master via WiFi"""
        if self.role != DeviceRole.NODE:
            raise ValueError("Only nodes can connect to master")

        try:
            print(f"Connecting to master at {master_ip}:{self.port}...")

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((master_ip, self.port))

            # Send device ID
            sock.send(self.device_id.encode('utf-8').ljust(64))

            self.connections['master'] = sock
            self.devices['master'] = DeviceInfo(
                device_id='master',
                bt_address=master_ip,
                role=DeviceRole.MASTER,
                last_seen=datetime.now()
            )

            print(f"Connected to master at {master_ip}")
            return True

        except Exception as e:
            print(f"Failed to connect to master: {e}")
            return False

    def send_data(self, data: bytes, target: str = 'master') -> bool:
        """Send encrypted data via WiFi"""
        if target not in self.connections:
            print(f"No connection to {target}")
            return False

        try:
            self.message_counter += 1

            secure_msg = SecureMessage(self.device_id, data, self.message_counter)
            packed = secure_msg.pack(self.encryption_mgr)

            sock = self.connections[target]
            sock.send(len(packed).to_bytes(4, 'big'))
            sock.send(packed)

            return True

        except Exception as e:
            print(f"Failed to send data to {target}: {e}")
            return False

    def broadcast(self, data: bytes) -> int:
        """Broadcast to all connected devices"""
        if self.role != DeviceRole.MASTER:
            raise ValueError("Only master can broadcast")

        success_count = 0
        for device_id in list(self.connections.keys()):
            if self.send_data(data, device_id):
                success_count += 1

        return success_count

    def get_connected_devices(self) -> List[str]:
        """Get connected device IDs"""
        return list(self.connections.keys())

    def get_network_stats(self) -> dict:
        """Get network statistics"""
        return {
            'device_id': self.device_id,
            'role': self.role.value,
            'protocol': 'WiFi',
            'connected_devices': len(self.connections),
            'total_messages_sent': self.message_counter,
            'devices': {
                device_id: {
                    'last_seen': info.last_seen.isoformat(),
                    'messages_received': info.message_counter
                }
                for device_id, info in self.devices.items()
            }
        }

    def stop(self):
        """Stop WiFi mesh network"""
        print("Stopping WiFi mesh network...")
        self.running = False

        for sock in self.connections.values():
            try:
                sock.close()
            except:
                pass
        self.connections.clear()

        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

        print("WiFi mesh network stopped")


# Alias for easier import
MeshNetwork = BluetoothMeshNetwork  # Default to Bluetooth


if __name__ == "__main__":
    print("Mesh network module - use examples/multi_device_scan.py for demo")