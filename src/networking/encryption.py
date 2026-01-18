"""
Encryption manager for secure multi-device communication
Implements AES-256-GCM encryption with key rotation and device authentication
"""

import os
import hmac
import hashlib
import json
from pathlib import Path
from typing import Tuple, Optional
from datetime import datetime, timedelta
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.hkdf import HKDF


class EncryptionManager:
    """Manages encryption, decryption, and key rotation for mesh network"""

    def __init__(self, device_id: str, key_file: Optional[str] = None):
        """
        Initialize encryption manager

        Args:
            device_id: Unique identifier for this device (e.g., "LIDAR_001")
            key_file: Path to encryption key file (default: ~/.lidar_mesh/key.bin)
        """
        self.device_id = device_id
        self.key_dir = Path.home() / ".lidar_mesh"
        self.key_dir.mkdir(exist_ok=True, mode=0o700)  # Secure directory

        if key_file:
            self.key_file = Path(key_file)
        else:
            self.key_file = self.key_dir / "key.bin"

        # Load or generate encryption key
        self.master_key = self._load_or_generate_key()

        # Generate device-specific secret for authentication
        self.device_secret = self._derive_device_secret()

        # Key rotation
        self.last_rotation = datetime.now()
        self.rotation_interval = timedelta(hours=24)  # Rotate every 24 hours

        # Message replay protection
        self.message_counters = {}  # device_id -> last_seen_counter

    def _load_or_generate_key(self) -> bytes:
        """Load existing key or generate new 256-bit key"""
        if self.key_file.exists():
            with open(self.key_file, 'rb') as f:
                key = f.read()
            if len(key) != 32:
                raise ValueError(f"Invalid key length: {len(key)} bytes (expected 32)")
            print(f"Loaded encryption key from {self.key_file}")
            return key
        else:
            # Generate new 256-bit key
            key = AESGCM.generate_key(bit_length=256)
            self._save_key(key)
            print(f"Generated new encryption key: {self.key_file}")
            return key

    def _save_key(self, key: bytes) -> None:
        """Save key to file with secure permissions"""
        with open(self.key_file, 'wb') as f:
            f.write(key)
        os.chmod(self.key_file, 0o600)  # Read/write for owner only

    def _derive_device_secret(self) -> bytes:
        """Derive device-specific secret from master key"""
        hkdf = HKDF(
            algorithm=hashes.SHA256(),
            length=32,
            salt=None,
            info=f"device_{self.device_id}".encode()
        )
        return hkdf.derive(self.master_key)

    def encrypt(self, plaintext: bytes) -> bytes:
        """
        Encrypt data with AES-256-GCM

        Args:
            plaintext: Data to encrypt

        Returns:
            Encrypted data: nonce (12 bytes) + ciphertext + auth_tag
        """
        # Check if key rotation needed
        if datetime.now() - self.last_rotation > self.rotation_interval:
            self.rotate_key()

        aesgcm = AESGCM(self.master_key)
        nonce = os.urandom(12)  # 96-bit nonce for GCM
        ciphertext = aesgcm.encrypt(nonce, plaintext, None)

        return nonce + ciphertext

    def decrypt(self, encrypted: bytes) -> bytes:
        """
        Decrypt data with AES-256-GCM

        Args:
            encrypted: Encrypted data (nonce + ciphertext)

        Returns:
            Decrypted plaintext

        Raises:
            cryptography.exceptions.InvalidTag: If authentication fails
        """
        if len(encrypted) < 12:
            raise ValueError("Encrypted data too short")

        nonce = encrypted[:12]
        ciphertext = encrypted[12:]

        aesgcm = AESGCM(self.master_key)
        plaintext = aesgcm.decrypt(nonce, ciphertext, None)

        return plaintext

    def sign_message(self, message: bytes, counter: int) -> bytes:
        """
        Create HMAC signature for message authentication

        Args:
            message: Message to sign
            counter: Message sequence counter for replay protection

        Returns:
            HMAC signature (32 bytes)
        """
        h = hmac.new(self.device_secret, digestmod=hashlib.sha256)
        h.update(self.device_id.encode())
        h.update(counter.to_bytes(8, 'big'))
        h.update(message)
        return h.digest()

    def verify_message(self, message: bytes, signature: bytes,
                      sender_id: str, counter: int) -> bool:
        """
        Verify HMAC signature and check for replay attacks

        Args:
            message: Received message
            signature: HMAC signature
            sender_id: Sender's device ID
            counter: Message sequence counter

        Returns:
            True if signature valid and no replay detected
        """
        # Derive sender's secret
        hkdf = HKDF(
            algorithm=hashes.SHA256(),
            length=32,
            salt=None,
            info=f"device_{sender_id}".encode()
        )
        sender_secret = hkdf.derive(self.master_key)

        # Compute expected signature
        h = hmac.new(sender_secret, digestmod=hashlib.sha256)
        h.update(sender_id.encode())
        h.update(counter.to_bytes(8, 'big'))
        h.update(message)
        expected = h.digest()

        # Constant-time comparison
        if not hmac.compare_digest(expected, signature):
            return False

        # Check for replay attack
        last_counter = self.message_counters.get(sender_id, -1)
        if counter <= last_counter:
            print(f"Replay attack detected from {sender_id}: counter {counter} <= {last_counter}")
            return False

        # Update counter
        self.message_counters[sender_id] = counter

        return True

    def rotate_key(self) -> None:
        """Rotate encryption key using HKDF key derivation"""
        print("Rotating encryption key...")

        # Derive new key from current key
        hkdf = HKDF(
            algorithm=hashes.SHA256(),
            length=32,
            salt=None,
            info=b'mesh_key_rotation'
        )
        new_key = hkdf.derive(self.master_key)

        # Save new key
        self.master_key = new_key
        self._save_key(new_key)

        # Update device secret
        self.device_secret = self._derive_device_secret()

        self.last_rotation = datetime.now()
        print(f"Key rotated successfully at {self.last_rotation}")

    def export_key(self, output_file: str) -> None:
        """
        Export master key to file for sharing with other devices

        Args:
            output_file: Path to save key (e.g., USB drive)

        Warning: Only share via secure physical medium (USB, not network)
        """
        output_path = Path(output_file)
        with open(output_path, 'wb') as f:
            f.write(self.master_key)
        os.chmod(output_path, 0o600)
        print(f"Key exported to {output_path}")
        print("WARNING: Transfer this file via USB to other devices")
        print("         Do NOT transmit over network!")

    def import_key(self, key_file: str) -> None:
        """
        Import master key from file

        Args:
            key_file: Path to key file from master device
        """
        with open(key_file, 'rb') as f:
            key = f.read()

        if len(key) != 32:
            raise ValueError(f"Invalid key length: {len(key)} bytes")

        self.master_key = key
        self._save_key(key)
        self.device_secret = self._derive_device_secret()
        print(f"Imported encryption key from {key_file}")

    def get_key_info(self) -> dict:
        """Get information about current encryption state"""
        return {
            'device_id': self.device_id,
            'key_file': str(self.key_file),
            'last_rotation': self.last_rotation.isoformat(),
            'next_rotation': (self.last_rotation + self.rotation_interval).isoformat(),
            'known_devices': list(self.message_counters.keys())
        }


class SecureMessage:
    """Encrypted message container"""

    def __init__(self, sender_id: str, data: bytes, counter: int):
        self.sender_id = sender_id
        self.data = data
        self.counter = counter
        self.timestamp = datetime.now()

    def pack(self, encryption_mgr: EncryptionManager) -> bytes:
        """
        Pack message with encryption and authentication

        Format: [sender_id_len(1)] [sender_id] [counter(8)] [signature(32)] [encrypted_data]
        """
        # Encrypt data
        encrypted_data = encryption_mgr.encrypt(self.data)

        # Create signature
        signature = encryption_mgr.sign_message(encrypted_data, self.counter)

        # Pack into bytes
        sender_bytes = self.sender_id.encode('utf-8')
        packed = bytes([len(sender_bytes)]) + sender_bytes
        packed += self.counter.to_bytes(8, 'big')
        packed += signature
        packed += encrypted_data

        return packed

    @staticmethod
    def unpack(packed: bytes, encryption_mgr: EncryptionManager) -> 'SecureMessage':
        """
        Unpack and verify encrypted message

        Raises:
            ValueError: If message format invalid or verification fails
        """
        if len(packed) < 42:  # Minimum: 1 + 1 + 8 + 32 = 42 bytes
            raise ValueError("Message too short")

        # Extract sender ID
        sender_id_len = packed[0]
        sender_id = packed[1:1+sender_id_len].decode('utf-8')

        # Extract counter
        counter_start = 1 + sender_id_len
        counter = int.from_bytes(packed[counter_start:counter_start+8], 'big')

        # Extract signature
        sig_start = counter_start + 8
        signature = packed[sig_start:sig_start+32]

        # Extract encrypted data
        encrypted_data = packed[sig_start+32:]

        # Verify signature
        if not encryption_mgr.verify_message(encrypted_data, signature, sender_id, counter):
            raise ValueError("Message signature verification failed")

        # Decrypt data
        try:
            decrypted_data = encryption_mgr.decrypt(encrypted_data)
        except Exception as e:
            raise ValueError(f"Decryption failed: {e}")

        return SecureMessage(sender_id, decrypted_data, counter)


if __name__ == "__main__":
    # Demo usage
    print("=== Encryption Manager Demo ===\n")

    # Create encryption manager for device 1
    mgr1 = EncryptionManager("LIDAR_001")
    print(f"Device 1 key info: {mgr1.get_key_info()}\n")

    # Test encryption/decryption
    plaintext = b"Hello from LIDAR_001! Here's some point cloud data..."
    print(f"Plaintext: {plaintext}")

    encrypted = mgr1.encrypt(plaintext)
    print(f"Encrypted: {len(encrypted)} bytes")

    decrypted = mgr1.decrypt(encrypted)
    print(f"Decrypted: {decrypted}")
    print(f"Match: {plaintext == decrypted}\n")

    # Test secure messaging
    print("=== Secure Message Demo ===\n")

    msg = SecureMessage("LIDAR_001", plaintext, counter=1)
    packed = msg.pack(mgr1)
    print(f"Packed message: {len(packed)} bytes")

    # Simulate receiving message
    received = SecureMessage.unpack(packed, mgr1)
    print(f"Received from: {received.sender_id}")
    print(f"Counter: {received.counter}")
    print(f"Data: {received.data}")
    print(f"Match: {plaintext == received.data}\n")

    # Test replay protection
    print("=== Replay Protection Demo ===\n")
    try:
        SecureMessage.unpack(packed, mgr1)  # Try to replay same message
        print("ERROR: Replay attack not detected!")
    except ValueError as e:
        print(f"✓ Replay attack blocked: {e}")