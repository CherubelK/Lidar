# Multi-Device Bluetooth Mesh Mapping System

## Overview

This system allows multiple LiDAR units to collaborate in creating a single large-scale map through Bluetooth mesh networking with encrypted communication.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Multi-Device Mesh Network                     │
│                                                                   │
│  Device 1 (Master)          Device 2 (Node)          Device 3    │
│  ┌──────────────┐          ┌──────────────┐       ┌──────────┐  │
│  │ Raspberry Pi │◄────BT───►│ Raspberry Pi │◄──BT─►│   RPi    │  │
│  │   + LiDAR    │          │   + LiDAR    │       │ + LiDAR  │  │
│  │   + GPS      │          │   + GPS      │       │ + GPS    │  │
│  └──────┬───────┘          └──────┬───────┘       └────┬─────┘  │
│         │                         │                     │        │
│         │  Encrypted Point Cloud  │                     │        │
│         │  + GPS Coordinates      │                     │        │
│         │  + Timestamps           │                     │        │
│         └────────────┬────────────┴─────────────────────┘        │
│                      │                                            │
│                      ▼                                            │
│            ┌──────────────────────┐                              │
│            │   Map Fusion Engine  │                              │
│            │  - ICP Registration  │                              │
│            │  - GPS Alignment     │                              │
│            │  - Conflict Resolve  │                              │
│            └──────────┬───────────┘                              │
│                       │                                           │
│                       ▼                                           │
│              ┌─────────────────┐                                 │
│              │  Unified 3D Map │                                 │
│              └─────────────────┘                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Bluetooth Technology Options

### Option 1: Bluetooth Classic (Recommended for Point Clouds)
**Technology**: Bluetooth 2.1 + EDR / Bluetooth 3.0 + HS

**Specifications**:
- Range: 10-100 meters (Class 1: 100m, Class 2: 10m)
- Throughput: 2-3 Mbps (Bluetooth 2.1 EDR)
- Max devices: 7 active slaves per master (piconet)
- Extended via scatternet: ~10-15 devices practical limit

**Pros**:
- High throughput for point cloud data
- Better range with external antenna
- Well-supported on Raspberry Pi
- Lower latency

**Cons**:
- Limited to 7 devices per piconet
- Complex scatternet management for >7 devices

### Option 2: Bluetooth Low Energy (BLE) Mesh
**Technology**: Bluetooth 5.0+ Mesh Networking

**Specifications**:
- Range: 50-200 meters (Bluetooth 5.0 extended range)
- Throughput: 1-2 Mbps (sufficient for compressed data)
- Max devices: 32,767 nodes theoretical (100+ practical)
- Mesh relay: Messages hop through intermediate nodes

**Pros**:
- True mesh topology (not just piconet)
- Unlimited device scaling via relay
- Self-healing network
- Lower power consumption
- Better for large deployments

**Cons**:
- Lower throughput (requires data compression)
- Higher latency due to multi-hop
- Requires Bluetooth 5.0+ hardware

### Option 3: WiFi Mesh (Alternative)
**Technology**: 802.11s or ESP-MESH

**Specifications**:
- Range: 50-100 meters outdoor
- Throughput: 50-150 Mbps
- Max devices: 100+ nodes
- Lower latency than BLE mesh

**Pros**:
- Much higher throughput
- Native on Raspberry Pi
- Better for real-time collaboration

**Cons**:
- Higher power consumption
- More complex setup

## Distance Limits & Range Extension

### Bluetooth Classic Range

| Class | Power | Indoor Range | Outdoor Range | Use Case |
|-------|-------|--------------|---------------|----------|
| Class 1 | 100 mW | 50-100m | 100-300m | Long-range scanning |
| Class 2 | 2.5 mW | 5-10m | 10-30m | Short-range only |
| Class 3 | 1 mW | 1-5m | 5-10m | Not suitable |

**Raspberry Pi Default**: Class 2 (10m range)

### Range Extension Methods

#### 1. External Bluetooth Antenna
```
Raspberry Pi USB Port
    │
    └──► USB Bluetooth Dongle (Class 1)
          │
          └──► External Antenna (2.4 GHz)
                │
                Range: 100-300 meters outdoor
```

**Recommended Hardware**:
- **ASUS USB-BT500** (Class 1, Bluetooth 5.0) - $20
- **TP-Link UB500** (Class 1, Bluetooth 5.0) - $15
- **External 2.4 GHz Antenna** (9 dBi gain) - $15

**Expected Range**:
- Indoor (walls): 50-100 meters
- Outdoor (clear): 200-400 meters
- With directional antenna: 500+ meters

#### 2. Bluetooth Mesh Relay
```
Device 1 ◄──50m──► Relay Node ◄──50m──► Device 2
                                            │
                                         50m│
                                            ▼
                                         Device 3

Total coverage: 100+ meters with 2-hop relay
```

**Relay Nodes**: Raspberry Pi Zero W ($10) with mesh forwarding

#### 3. WiFi Mesh (Best for Large Areas)
```
Device 1 ◄──WiFi──► Device 2 ◄──WiFi──► Device 3
  (100m range)         (Relay)           (100m)

Total coverage: 200+ meters, unlimited hops
```

### Practical Distance Recommendations

| Scenario | Technology | Max Distance | Devices | Setup Cost |
|----------|-----------|--------------|---------|------------|
| **Small room (indoor)** | BT Classic | 10m | 2-4 | $0 (built-in) |
| **Large building** | BLE Mesh | 50m | 5-10 | $100 (dongles) |
| **Outdoor field** | BT Class 1 + Antenna | 300m | 2-7 | $150 (dongles+antennas) |
| **Large outdoor area** | WiFi Mesh | 500m+ | 10+ | $200 (mesh APs) |

## Encryption & Security

### Encryption Standards

#### 1. Bluetooth Classic Security
```
┌────────────────────────────────────────┐
│  Bluetooth Pairing (E0 Stream Cipher) │
│  + AES-128 CCM Encryption Layer        │
│  + Application-Level AES-256-GCM       │
└────────────────────────────────────────┘
```

**Security Layers**:
1. **Link-Level**: Bluetooth pairing with PIN/passkey
2. **Transport**: AES-128 CCM (Bluetooth Secure Connections)
3. **Application**: AES-256-GCM for point cloud data

**Implementation**:
```python
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
import os

# Generate 256-bit key (shared across all devices)
encryption_key = AESGCM.generate_key(bit_length=256)

# Encrypt point cloud before transmission
def encrypt_data(plaintext: bytes, key: bytes) -> bytes:
    aesgcm = AESGCM(key)
    nonce = os.urandom(12)  # 96-bit nonce
    ciphertext = aesgcm.encrypt(nonce, plaintext, None)
    return nonce + ciphertext  # Prepend nonce

# Decrypt received data
def decrypt_data(encrypted: bytes, key: bytes) -> bytes:
    aesgcm = AESGCM(key)
    nonce = encrypted[:12]
    ciphertext = encrypted[12:]
    return aesgcm.decrypt(nonce, ciphertext, None)
```

#### 2. BLE Mesh Security
```
┌───────────────────────────────────────────┐
│  Network Layer: AES-128 CCM               │
│  + Application Key: AES-128 CCM           │
│  + Device Key: AES-128 CCM (per-device)   │
└───────────────────────────────────────────┘
```

**BLE Mesh Security Model**:
- **Network Key (NetKey)**: Encrypts all mesh traffic
- **Application Key (AppKey)**: Encrypts application data
- **Device Key (DevKey)**: Unique per device for provisioning

**Key Features**:
- Forward secrecy (keys rotated)
- Replay protection (sequence numbers)
- Privacy (address obfuscation)

### Key Management

#### Secure Key Distribution
```
Master Device (Device 1)
    │
    ├──► Generate shared 256-bit key on first boot
    │
    ├──► Store in encrypted file: /home/pi/.lidar_mesh/key.enc
    │
    └──► Distribute via:
         Option 1: Physical USB transfer (most secure)
         Option 2: QR code scan (moderate security)
         Option 3: Bluetooth pairing + Diffie-Hellman (automated)
```

**Recommended: USB Key Transfer**
```bash
# On master device
python mesh_keygen.py --generate --output /media/usb/mesh_key.bin

# On each node device
python mesh_keygen.py --import /media/usb/mesh_key.bin
```

#### Key Rotation
```python
# Rotate keys every 24 hours for enhanced security
def rotate_keys(old_key: bytes) -> bytes:
    # Derive new key using HKDF
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.kdf.hkdf import HKDF

    hkdf = HKDF(
        algorithm=hashes.SHA256(),
        length=32,
        salt=None,
        info=b'mesh_key_rotation'
    )
    new_key = hkdf.derive(old_key)
    return new_key
```

### Authentication

#### Device Authentication
```python
# Each device has unique ID + shared secret
device_id = "LIDAR_001"  # Unique identifier
device_secret = os.urandom(32)  # Generated during provisioning

# Create HMAC for message authentication
import hmac
import hashlib

def authenticate_message(message: bytes, device_id: str, secret: bytes) -> bytes:
    h = hmac.new(secret, digestmod=hashlib.sha256)
    h.update(device_id.encode())
    h.update(message)
    return h.digest()

# Verify received message
def verify_message(message: bytes, signature: bytes, device_id: str, secret: bytes) -> bool:
    expected = authenticate_message(message, device_id, secret)
    return hmac.compare_digest(expected, signature)
```

## Data Compression

Point clouds are large - compression is essential for Bluetooth transmission.

### Compression Pipeline
```
Raw Point Cloud (10 MB)
    │
    ├──► Voxel Downsampling (0.05m grid) → 1 MB
    │
    ├──► Quantization (float32 → int16) → 0.5 MB
    │
    ├──► LZ4 Compression → 0.2 MB
    │
    └──► Encrypted Package → 0.2 MB + overhead
```

**Implementation**:
```python
import lz4.frame
import numpy as np

def compress_pointcloud(points: np.ndarray, voxel_size: float = 0.05) -> bytes:
    # 1. Voxel downsample
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    downsampled = pcd.voxel_down_sample(voxel_size)

    # 2. Quantize to int16 (2-byte integers)
    points_ds = np.asarray(downsampled.points)
    min_coords = points_ds.min(axis=0)
    scale = (points_ds.max(axis=0) - min_coords) / 65535
    quantized = ((points_ds - min_coords) / scale).astype(np.uint16)

    # 3. Store metadata + quantized data
    metadata = np.array([min_coords[0], min_coords[1], min_coords[2],
                         scale[0], scale[1], scale[2]], dtype=np.float32)
    data = np.concatenate([metadata.tobytes(), quantized.tobytes()])

    # 4. LZ4 compress
    compressed = lz4.frame.compress(data)

    return compressed

def decompress_pointcloud(compressed: bytes) -> np.ndarray:
    # Reverse the process
    data = lz4.frame.decompress(compressed)

    # Extract metadata
    metadata = np.frombuffer(data[:24], dtype=np.float32)
    min_coords = metadata[:3]
    scale = metadata[3:]

    # Dequantize
    quantized = np.frombuffer(data[24:], dtype=np.uint16).reshape(-1, 3)
    points = quantized.astype(np.float32) * scale + min_coords

    return points
```

**Compression Ratio**: 50:1 typical (10 MB → 200 KB)

## Network Topology

### Star Topology (Bluetooth Classic)
```
       Device 2
          │
          │ (BT)
          │
Device 3──┼──Device 1 (Master)──┼──Device 4
          │                     │
          │                     │
       Device 5              Device 6

Limit: 7 devices (master + 6 slaves)
Range: 10-100m from master
```

**Pros**:
- Simple to implement
- Low latency
- Master aggregates all data

**Cons**:
- Single point of failure (master)
- Distance limited by master range

### Mesh Topology (BLE Mesh or WiFi)
```
Device 1 ──── Device 2 ──── Device 3
   │             │             │
   │             │             │
Device 4 ──── Device 5 ──── Device 6
   │             │             │
   │             │             │
Device 7 ──── Device 8 ──── Device 9

Unlimited devices, messages relay through network
```

**Pros**:
- Unlimited scaling
- Self-healing (alternate routes)
- Extended range via relay

**Cons**:
- Higher latency (multi-hop)
- More complex synchronization

## Map Merging Strategy

### 1. GPS-Based Alignment (Outdoor)
```python
# If all devices have GPS
def merge_maps_gps(maps: List[Dict]) -> np.ndarray:
    merged_points = []

    for device_map in maps:
        points = device_map['points']  # Nx3 array
        gps_origin = device_map['gps_origin']  # (lat, lon, alt)

        # Convert to global UTM coordinates
        global_points = convert_to_global_utm(points, gps_origin)
        merged_points.append(global_points)

    # Concatenate all points
    return np.vstack(merged_points)
```

### 2. ICP Registration (Indoor - No GPS)
```python
import open3d as o3d

def merge_maps_icp(maps: List[np.ndarray]) -> np.ndarray:
    # Start with first map as reference
    merged = maps[0]

    for i in range(1, len(maps)):
        # Register new map to merged map
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(maps[i])

        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(merged)

        # ICP alignment
        threshold = 0.5  # 50cm
        trans_init = np.eye(4)

        reg = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
        )

        # Transform and merge
        source.transform(reg.transformation)
        merged = np.vstack([merged, np.asarray(source.points)])

    return merged
```

### 3. Hybrid GPS + ICP (Best Approach)
```python
def merge_maps_hybrid(maps: List[Dict]) -> np.ndarray:
    # Use GPS for coarse alignment
    gps_aligned = []
    for device_map in maps:
        if device_map['has_gps']:
            global_points = convert_to_global_utm(
                device_map['points'],
                device_map['gps_origin']
            )
            gps_aligned.append(global_points)
        else:
            gps_aligned.append(device_map['points'])

    # Refine with ICP
    merged = gps_aligned[0]
    for i in range(1, len(gps_aligned)):
        # ICP refinement with GPS as initial guess
        merged = icp_refine(merged, gps_aligned[i])

    return merged
```

## Bandwidth Requirements

### Point Cloud Data Rates

**Uncompressed**:
- Points per scan: 48,000 (Unitree L2 @ 10Hz decimated to 5Hz)
- Bytes per point: 12 (3x float32)
- Scan size: 576 KB
- Rate @ 5 Hz: 2.88 MB/s

**Compressed (50:1)**:
- Scan size: ~12 KB
- Rate @ 5 Hz: 60 KB/s = 480 Kbps

**Bluetooth Capacity**:
- Bluetooth Classic (EDR): 2-3 Mbps → Can handle 4-6 devices @ 5Hz
- BLE: 1 Mbps → Can handle 2 devices @ 5Hz (or reduce scan rate)
- WiFi: 50+ Mbps → Can handle 100+ devices easily

### Transmission Strategies

#### 1. Real-Time Streaming (Low Latency)
```python
# Send every scan immediately
while scanning:
    points = lidar.get_scan()
    compressed = compress_pointcloud(points)
    encrypted = encrypt_data(compressed, mesh_key)
    mesh.broadcast(encrypted)
    time.sleep(0.2)  # 5 Hz
```

**Use case**: Real-time collaborative mapping
**Bandwidth**: 480 Kbps per device

#### 2. Batched Transfer (Efficient)
```python
# Accumulate 10 scans, send batch
batch = []
while scanning:
    points = lidar.get_scan()
    batch.append(points)

    if len(batch) >= 10:
        merged_batch = np.vstack(batch)
        compressed = compress_pointcloud(merged_batch)
        encrypted = encrypt_data(compressed, mesh_key)
        mesh.broadcast(encrypted)
        batch = []

    time.sleep(0.2)
```

**Use case**: Battery-constrained devices
**Bandwidth**: 48 Kbps average (10x reduction)

#### 3. Post-Processing Merge (Offline)
```python
# Save locally, merge after scanning
while scanning:
    points = lidar.get_scan()
    local_map.append(points)

# After scan complete
final_map = np.vstack(local_map)
compressed = compress_pointcloud(final_map)
encrypted = encrypt_data(compressed, mesh_key)
mesh.send_to_master(encrypted)
```

**Use case**: Large-scale mapping with post-processing
**Bandwidth**: One-time transfer after scanning

## Power Consumption

| Component | Bluetooth Classic | BLE Mesh | WiFi Mesh |
|-----------|------------------|----------|-----------|
| Raspberry Pi 4 | 2.5W (idle) | 2.5W (idle) | 2.8W (WiFi active) |
| LiDAR | 15W | 15W | 15W |
| GPS | 0.5W | 0.5W | 0.5W |
| BT Radio | 0.5W | 0.1W | - |
| WiFi Radio | - | - | 1.0W |
| **Total** | **18.5W** | **18.1W** | **19.3W** |

**Battery Life** (5000 mAh @ 12V = 60 Wh):
- Bluetooth: 3.2 hours
- BLE Mesh: 3.3 hours
- WiFi Mesh: 3.1 hours

## Implementation Recommendation

### For Your Use Case

Based on your requirements, I recommend:

**Option A: Small Deployment (2-4 Devices, <50m)**
- **Technology**: Bluetooth Classic (built-in)
- **Topology**: Star (one master)
- **Range**: 10-50m with Class 1 dongle
- **Cost**: $40 (USB dongles)
- **Setup time**: 2 hours

**Option B: Medium Deployment (5-10 Devices, 50-200m)**
- **Technology**: BLE Mesh + External Antennas
- **Topology**: Mesh with relay
- **Range**: 100-200m
- **Cost**: $200 (dongles + antennas)
- **Setup time**: 4 hours

**Option C: Large Deployment (10+ Devices, 200m+)**
- **Technology**: WiFi Mesh (ESP-MESH or 802.11s)
- **Topology**: Full mesh
- **Range**: 500m+ with repeaters
- **Cost**: $300 (mesh routers)
- **Setup time**: 8 hours

## Security Best Practices

1. ✓ **Always encrypt** all transmitted data (AES-256-GCM)
2. ✓ **Use unique device IDs** for authentication
3. ✓ **Rotate keys** every 24 hours minimum
4. ✓ **Disable pairing** after initial setup
5. ✓ **Use MAC address filtering** (whitelist known devices)
6. ✓ **Enable Bluetooth Secure Connections** (AES-128 CCM)
7. ✓ **Store keys encrypted** on filesystem
8. ✗ **Never transmit keys** over unencrypted channels
9. ✓ **Use SSH** for remote management (disable Bluetooth management)
10. ✓ **Implement replay protection** (message sequence numbers)

## Next Steps

I'll now implement:
1. Bluetooth mesh networking module
2. Encryption/decryption layer
3. Map merging engine
4. Example multi-device scanning script
5. Setup and configuration guide

Would you like me to proceed with the implementation?