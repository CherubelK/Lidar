# GPS RTK Setup Guide - SparkFun GPS-RTK2 + Raspberry Pi

## Hardware Required

1. **SparkFun GPS-RTK2 Board - ZED-F9P** ($274.95)
   - Buy: https://www.sparkfun.com/products/15136
   - Includes: ZED-F9P module, SMA connector

2. **GNSS Multi-Band Antenna** ($69.95)
   - SparkFun GNSS Multi-Band L1/L2 Surveying Antenna (TNC)
   - Buy: https://www.sparkfun.com/products/17751
   - OR: Interface Cable SMA to TNC ($4.50)

3. **GNSS Antenna Cable** (if needed)
   - SMA to U.FL cable included with board

4. **Raspberry Pi** (any model)
   - You likely already have this

## Physical Connection

### Option 1: UART Connection (Recommended for permanent setup)

```
SparkFun GPS-RTK2          Raspberry Pi GPIO
-----------------          ------------------
3.3V         ---------->   Pin 1 (3.3V Power)
GND          ---------->   Pin 6 (Ground)
TX (UART1)   ---------->   Pin 10 (GPIO 15 - RXD)
RX (UART1)   ---------->   Pin 8 (GPIO 14 - TXD)
```

**Wiring Diagram:**
```
GPS-RTK2 Board
┌─────────────────┐
│  3V3  GND TX RX │
└───┬───┬───┬───┬─┘
    │   │   │   │
    │   │   │   └─────> Pin 8 (GPIO 14)
    │   │   └─────────> Pin 10 (GPIO 15)
    │   └─────────────> Pin 6 (GND)
    └─────────────────> Pin 1 (3.3V)

Raspberry Pi GPIO Header
```

### Option 2: USB Connection (Easiest for testing)

1. Connect GPS-RTK2 USB-C port to Raspberry Pi USB port
2. GPS will appear as `/dev/ttyACM0` or `/dev/ttyUSB0`

## Raspberry Pi Setup

### 1. Enable UART (for UART connection)

```bash
# Edit boot configuration
sudo nano /boot/config.txt

# Add these lines at the end:
enable_uart=1
dtoverlay=disable-bt

# Save and exit (Ctrl+X, Y, Enter)

# Reboot
sudo reboot
```

### 2. Install Required Software

```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install Python libraries
pip install pyserial pynmea2 pyubx2 utm

# Optional: GPS testing tools
sudo apt-get install gpsd gpsd-clients
```

### 3. Test GPS Connection

#### Quick Test (USB):
```bash
# List USB devices
lsusb

# Check serial ports
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*

# Read raw GPS data
cat /dev/ttyACM0
# You should see NMEA sentences like $GNGGA, $GNRMC, etc.
```

#### Quick Test (UART):
```bash
# Read from UART
cat /dev/serial0
# You should see NMEA sentences
```

#### Python Test:
```bash
python src/positioning/gps_rtk.py
```

## RTK Correction Setup (for cm accuracy)

### What is RTK?

RTK (Real-Time Kinematic) provides centimeter-level accuracy by using correction data from a base station or NTRIP service.

**Without RTK**: 2-5 meter accuracy (standard GPS)
**With RTK**: 1-2 centimeter accuracy!

### Option 1: Free NTRIP Services

NTRIP provides RTK corrections via internet. Many regions have free services:

**United States:**
- UNAVCO: https://www.unavco.org/data/gps-gnss/real-time/real-time.html
- CORS Network: https://geodesy.noaa.gov/CORS/

**Setup:**
1. Find nearest NTRIP caster (within 50km)
2. Get mount point, username, password
3. Configure ZED-F9P with u-center software or python

### Option 2: Base Station (Advanced)

Set up your own RTK base station with another ZED-F9P module:
- Place base station at known location
- Broadcast corrections to rover (your scanning unit)
- Range: ~10km with radio link

## Configure ZED-F9P

### Using u-center (Windows Software)

1. Download u-center from u-blox: https://www.u-blox.com/en/product/u-center
2. Connect GPS via USB
3. Configure:
   - Baud rate: 38400 (or 115200 for faster)
   - Enable UBX protocol
   - Configure NTRIP client settings
   - Save configuration to module

### Using Python (pyubx2)

```python
from pyubx2 import UBXMessage, SET
import serial

# Connect
ser = serial.Serial('/dev/ttyACM0', 38400)

# Set baud rate to 115200
msg = UBXMessage('CFG', 'CFG-PRT', SET,
                 portID=1, baudRate=115200)
ser.write(msg.serialize())

# Enable RTK
# ... (more configuration)
```

## Testing the Integration

### 1. Test GPS Module Alone

```bash
cd examples
python ../src/positioning/gps_rtk.py
```

**Expected output:**
```
Waiting for GPS fix...
  Satellites: 8, Quality: GPS Fix
GPS fix acquired! Quality: GPS Fix

GPS Position:
  Latitude: 40.12345678°
  Longitude: -105.12345678°
  Altitude: 1650.50m
  Fix Quality: GPS Fix
  Satellites: 12
  HDOP: 0.85
```

### 2. Test GPS + LiDAR Integration

```bash
python examples/gps_lidar_scan.py
```

## Troubleshooting

### No GPS Fix

**Problem**: GPS doesn't get a fix

**Solutions**:
1. ✓ Check antenna is properly connected (SMA connector tight)
2. ✓ Place antenna outdoors with clear sky view
3. ✓ Wait 2-5 minutes for initial fix (cold start)
4. ✓ Check GPS has power (LED should be blinking)

### Serial Port Not Found

**Problem**: `/dev/serial0` or `/dev/ttyACM0` not found

**Solutions**:
1. ✓ Check USB connection: `lsusb`
2. ✓ Check UART is enabled: `ls -l /dev/serial0`
3. ✓ Try different port: `/dev/ttyUSB0`, `/dev/ttyACM0`
4. ✓ Add user to dialout group: `sudo usermod -a -G dialout $USER`

### Permission Denied

**Problem**: `Permission denied: '/dev/ttyACM0'`

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Reboot or re-login
sudo reboot
```

### No RTK Fix

**Problem**: GPS fix but no RTK

**Solutions**:
1. ✓ Check NTRIP connection (needs internet)
2. ✓ Verify NTRIP credentials are correct
3. ✓ Ensure base station is within 50km
4. ✓ Wait 5-10 minutes for RTK convergence
5. ✓ Check correction data age (<30 seconds)

## Performance Benchmarks

**GPS Update Rate**: 1-10 Hz (ZED-F9P capable of 10Hz)
**LiDAR Update Rate**: ~5 Hz with KISS-ICP
**Fusion Rate**: 5 Hz (limited by LiDAR processing)

**Accuracy**:
- GPS only: 2-5 meters
- GPS + DGPS: 1-2 meters
- GPS + RTK Float: 10-20 cm
- GPS + RTK Fixed: 1-2 cm

**Combined GPS + LiDAR**:
- Outdoor trail mapping: 5-10 cm accuracy
- With RTK: 2-5 cm accuracy

## Example Scan Workflow

### Outdoor Trail Scanning

```bash
# 1. Start GPS + LiDAR scan
python examples/gps_lidar_scan.py

# 2. Walk along trail slowly (~1 m/s)
#    GPS updates position every 1 second
#    LiDAR fills in details between updates

# 3. View results
#    http://localhost:8000/viewer.html
```

### Park or Outdoor Area

```bash
# 1. Set up in center of area
# 2. Wait for RTK fix (best accuracy)
# 3. Walk systematic pattern
# 4. GPS provides absolute position
# 5. KISS-ICP provides detailed movements
```

## Cost Breakdown

| Item | Cost |
|------|------|
| SparkFun GPS-RTK2 (ZED-F9P) | $274.95 |
| GNSS Multi-Band Antenna | $69.95 |
| SMA to TNC Cable (if needed) | $4.50 |
| **Total** | **~$350** |

**NTRIP Service**: Free (or $10-50/month for premium)

## Comparison: GPS vs UWB

| Feature | GPS RTK | UWB |
|---------|---------|-----|
| **Outdoor** | ✅ Excellent | ❌ Limited range |
| **Indoor** | ❌ Doesn't work | ✅ Excellent |
| **Accuracy** | 1-2 cm (RTK) | 10-30 cm |
| **Setup** | None (uses satellites) | Requires anchors |
| **Cost** | ~$350 one-time | ~$400 + setup |
| **Range** | Global | ~50m |

**Recommendation**:
- **Outdoor work**: Use GPS RTK
- **Indoor work**: Use UWB (or KISS-ICP alone)
- **Both**: Get both systems!

## Next Steps

1. **Order hardware**: SparkFun GPS-RTK2 + Antenna
2. **Set up UART connection** on Raspberry Pi
3. **Test GPS module** with test script
4. **Find NTRIP service** in your area
5. **Configure RTK** for cm-level accuracy
6. **Run outdoor scan** and compare with indoor KISS-ICP results

The GPS RTK integration is ready to use once you have the hardware!