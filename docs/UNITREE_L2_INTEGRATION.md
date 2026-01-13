# Unitree L2 LiDAR Integration Guide

This guide provides detailed instructions for integrating the Unitree L2 4D LiDAR sensor with your Python project.

## Table of Contents
1. [Hardware Setup](#hardware-setup)
2. [Network Configuration](#network-configuration)
3. [SDK Options](#sdk-options)
4. [Python Implementation](#python-implementation)
5. [Integration Steps](#integration-steps)

## Hardware Setup

### Unitree L2 Specifications
- **Range**: Up to 30 meters
- **Points per scan**: ~5,200 points
- **Rings**: 18 vertical rings
- **Field of View**: 360° horizontal, configurable vertical
- **Connection**: Ethernet (UDP) or Serial Port
- **IMU**: Built-in IMU for orientation data

### Physical Connection
1. Connect the Unitree L2 to your computer via Ethernet cable
2. Power the sensor using the provided power supply
3. Ensure LED indicators show the sensor is powered on

## Network Configuration

### Default Sensor Settings
- **Sensor IP**: 192.168.1.62
- **Sensor Port**: 6101 (outgoing data)
- **Host IP**: 192.168.1.2 (your computer)
- **Host Port**: 6201 (receiving data)
- **Netmask**: 255.255.255.0
- **Gateway**: 192.168.1.1

### Configure Your Computer's Network

**Windows:**
1. Open Network & Internet Settings
2. Click "Change adapter options"
3. Right-click your Ethernet adapter → Properties
4. Select "Internet Protocol Version 4 (TCP/IPv4)"
5. Configure as follows:
   - IP address: `192.168.1.2`
   - Subnet mask: `255.255.255.0`
   - Default gateway: `192.168.1.1`

**Linux/Mac:**
```bash
sudo ifconfig <interface> 192.168.1.2 netmask 255.255.255.0
```

### Test Connection
```bash
ping 192.168.1.62
```
You should get responses if the sensor is connected properly.

## SDK Options

### Option 1: Official Unitree SDK (C++)
**Repository**: [unitreerobotics/unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2)

**Pros**:
- Official support
- Most stable and complete
- Best performance

**Cons**:
- C++ only (requires compilation)
- More complex to use from Python
- Requires CMake build system

**Use when**: You need maximum performance or official support

### Option 2: Third-Party Python Implementation
**Repository**: [dilohn/unitree-L2-lidar](https://github.com/dilohn/unitree-L2-lidar)

**Pros**:
- Pure Python (Windows compatible)
- Direct UDP parsing
- Includes visualization examples

**Cons**:
- Unofficial/community-maintained
- Limited to specific packet modes
- Less tested than official SDK

**Use when**: You want a pure Python solution or are on Windows

### Option 3: Custom Python UDP Parser (This Project)
**Location**: `src/lidar_interface/unitree_l2_udp.py` (to be created)

**Pros**:
- Integrated with your project
- No external dependencies (besides NumPy)
- Full control over implementation

**Cons**:
- Requires understanding of Unitree protocol
- More development work

**Use when**: You want complete control and minimal dependencies

## Python Implementation

### Required Configuration

The Unitree L2 must be configured for:
- **Packet Type**: 3D point packets
- **IMU**: Disabled (or handle separately)
- **Mode**: Normal (not negative angle mode)

### UDP Packet Structure

Each UDP packet contains:
1. **Header**: Packet metadata and timestamps
2. **Calibration Data**: Sensor calibration parameters
3. **Point Data**: Array of 3D points with intensity

### Point Data Format

Each point contains:
- `x`, `y`, `z`: 3D coordinates (meters)
- `intensity`: Reflection intensity value
- `timestamp`: Point timestamp
- `ring`: Vertical ring number (0-17)

## Integration Steps

### Step 1: Install Dependencies

If using third-party Python implementation:
```bash
git clone https://github.com/dilohn/unitree-L2-lidar
cd unitree-L2-lidar
pip install -r requirements.txt  # If available
```

### Step 2: Test Basic Connection

Create a test script:
```python
import socket
import struct

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('192.168.1.2', 6201))
sock.settimeout(5.0)

print("Listening for Unitree L2 data...")

try:
    data, addr = sock.recvfrom(4096)
    print(f"Received {len(data)} bytes from {addr}")
    print("Connection successful!")
except socket.timeout:
    print("Timeout - check sensor configuration and network settings")
finally:
    sock.close()
```

### Step 3: Update Project Configuration

Edit `config/lidar_config.yaml`:
```yaml
lidar:
  ip_address: "192.168.1.62"
  port: 6101
  host_ip: "192.168.1.2"
  host_port: 6201
  protocol: "udp"
```

### Step 4: Implement UDP Receiver

See `src/lidar_interface/unitree_l2_udp.py` for the full implementation.

Key methods to implement:
- `connect()`: Set up UDP socket
- `receive_packet()`: Receive raw UDP packet
- `parse_packet()`: Parse binary data to point cloud
- `get_point_cloud()`: Return Nx3 NumPy array

### Step 5: Integrate with Existing Code

Update `src/lidar_interface/unitree_l2.py`:
```python
from .unitree_l2_udp import UnitreeL2UDP

class UnitreeL2LiDAR:
    def __init__(self, config):
        self.config = config
        self.udp_receiver = UnitreeL2UDP(
            sensor_ip=config.ip_address,
            sensor_port=config.port,
            host_ip=config.host_ip,
            host_port=config.host_port
        )

    def connect(self):
        return self.udp_receiver.connect()

    def get_point_cloud(self):
        return self.udp_receiver.get_point_cloud()
```

## Sensor Configuration

### Using Official Configuration Tool

Unitree provides a configuration tool to change sensor settings:
- Access via Ethernet at `http://192.168.1.62` (check manual for exact URL)
- Or use provided Windows/Linux configuration software

### Important Settings

1. **Work Mode**: Set to Normal (Mode 0)
2. **Packet Type**: Set to 3D Points
3. **IMU Output**: Disable if only need point cloud
4. **Frame Rate**: Typically 10Hz
5. **Range Filtering**: Configure min/max range as needed

## Troubleshooting

### No Data Received
- Check network configuration (IP address must match)
- Verify sensor power and LED status
- Confirm firewall isn't blocking UDP port 6201
- Test with `ping 192.168.1.62`

### Incomplete/Corrupted Data
- Check packet type is set to 3D points
- Verify sensor firmware is up to date
- Ensure network cable is good quality
- Try reducing network traffic on the interface

### Low Frame Rate
- Check CPU usage (parsing is intensive)
- Use downsampling in processing pipeline
- Consider multithreading for parsing

### Wrong Coordinate System
- Unitree L2 uses right-handed coordinate system
- X: forward, Y: left, Z: up
- May need to transform for your application

## Next Steps

1. Review the third-party Python implementation for packet parsing examples
2. Create `unitree_l2_udp.py` based on the protocol
3. Test with real sensor hardware
4. Calibrate and validate point cloud data
5. Integrate with existing processing pipeline

## Resources

- [Official SDK Repository](https://github.com/unitreerobotics/unilidar_sdk2)
- [Third-Party Python Implementation](https://github.com/dilohn/unitree-L2-lidar)
- [Unitree Official Website](https://www.unitree.com/mobile/L2/)
- [User Manual (PDF)](https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf)

## Contact & Support

For official support, contact Unitree Robotics through their website or GitHub issues on the official SDK repository.