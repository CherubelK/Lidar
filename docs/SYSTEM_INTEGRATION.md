# Complete System Integration Guide

## System Overview

Your complete LiDAR scanning system will have these components:

```
┌─────────────────────────────────────────────────────────────────┐
│                    COMPLETE SCANNING SYSTEM                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐  │
│  │ Unitree L2   │◄────►│ Raspberry Pi │◄────►│ GPS-RTK2     │  │
│  │ LiDAR        │ UDP  │              │ UART │ (ZED-F9P)    │  │
│  └──────────────┘      └──────────────┘      └──────────────┘  │
│         │                     │                      │          │
│         │                     │                      │          │
│    Power (12V)           Power (5V)            Power (3.3V)     │
│         │                     │                      │          │
│         └─────────────────────┴──────────────────────┘          │
│                               │                                 │
│                      ┌────────▼────────┐                        │
│                      │ Power Supply    │                        │
│                      │ 12V + 5V output │                        │
│                      └────────┬────────┘                        │
│                               │                                 │
│                          Wall Outlet                            │
└─────────────────────────────────────────────────────────────────┘
```

## Component Requirements

### 1. Unitree L2 LiDAR Sensor

**Power Requirements:**
- Voltage: 12V DC
- Current: 1-2A (typical)
- Power consumption: 12-24W
- Connector: DC barrel jack (exact spec from Unitree)

**Data Connection:**
- Protocol: UDP over Ethernet
- IP: 192.168.1.62 (LiDAR)
- Target: 192.168.1.2 (your device)
- Port: 6101 → 6201
- Cable: Ethernet (RJ45)

### 2. Raspberry Pi

**Model Options:**
- Raspberry Pi 4B (4GB/8GB) - Recommended
- Raspberry Pi 3B+ - Budget option
- Raspberry Pi 5 - Latest (if available)

**Power Requirements:**
- Voltage: 5V DC
- Current: 3A (Pi 4), 2.5A (Pi 3)
- Power consumption: 15W max
- Connector: USB-C (Pi 4) or Micro-USB (Pi 3)

**Connections:**
- Ethernet: To LiDAR sensor
- UART (GPIO): To GPS module
- USB: Optional (for GPS via USB)
- WiFi: Optional (for NTRIP internet connection)

### 3. SparkFun GPS-RTK2 (ZED-F9P)

**Power Requirements:**
- Voltage: 3.3V DC (from Pi GPIO) or 5V (via USB)
- Current: 100-150mA
- Power consumption: 0.5W

**Connections:**
- UART: To Raspberry Pi GPIO
- OR USB: To Raspberry Pi USB port
- Antenna: SMA or U.FL connector

**GNSS Antenna:**
- Type: Active GNSS antenna (L1/L2 bands)
- Power: 3.3V from GPS module
- Connector: SMA or TNC
- Cable: Low-loss coax (RG174 or better)

## Complete Wiring Diagram

### Physical Layout

```
                    GNSS Antenna (on roof/outside)
                           │
                           │ (SMA cable)
                           │
┌──────────────────────────┼────────────────────────────────┐
│                          ▼                                 │
│  ┌───────────────────────────────────┐                    │
│  │    SparkFun GPS-RTK2 Board       │                    │
│  │         (ZED-F9P)                │                    │
│  │                                   │                    │
│  │  [SMA]  3V3  GND  TX   RX  [USB] │                    │
│  └────┬─────┬────┬────┬────┬─────┬──┘                    │
│       │     │    │    │    │     │                        │
│       │     │    │    │    │     └──────────┐             │
│       │     │    │    │    │                │             │
│  ┌────┴─────┴────┴────┴────┴──────┐    ┌───▼──────┐      │
│  │   Raspberry Pi 4 GPIO Header   │    │   USB    │      │
│  │                                 │    │   Port   │      │
│  │  Pin Layout:                    │    └──────────┘      │
│  │  1  [3.3V] ●─────────────────┐  │                      │
│  │  6  [GND]  ●──────────────┐  │  │                      │
│  │  8  [TXD]  ●──────────┐   │  │  │                      │
│  │  10 [RXD]  ●───────┐  │   │  │  │                      │
│  │                    │  │   │  │  │                      │
│  │  [Ethernet Port]◄──┼──┼───┼──┼──┼─────┐                │
│  └────────────────────┼──┼───┼──┼──┼─────┼───┐            │
│                       │  │   │  │  │     │   │            │
│                    ┌──▼──▼───▼──▼──▼┐    │   │            │
│                    │ 5V Power Input │    │   │            │
│                    │   (USB-C/uUSB) │    │   │            │
│                    └────────┬───────┘    │   │            │
│                             │            │   │            │
│  ┌──────────────────────────┼────────────┼───┼───────┐    │
│  │  Unitree L2 LiDAR        │            │   │       │    │
│  │                          │            │   │       │    │
│  │  [Ethernet Port] ◄───────┘            │   │       │    │
│  │  [12V Power Input] ◄──────────────────┘   │       │    │
│  └────────────────────────────────────────────┘       │    │
│                                                       │    │
│  ┌────────────────────────────────────────────────────┘    │
│  │  Dual Voltage Power Supply                             │
│  │  ┌──────────────┐  ┌──────────────┐                    │
│  │  │  12V Output  │  │   5V Output  │                    │
│  │  │   2A-3A      │  │     3A       │                    │
│  │  └──────┬───────┘  └──────┬───────┘                    │
│  │         │                 │                            │
│  │         │                 │                            │
│  │  ┌──────┴─────────────────┴───────┐                    │
│  │  │        AC Input 110-240V       │                    │
│  │  └────────────────────────────────┘                    │
│  │                  │                                      │
│  └──────────────────┼──────────────────────────────────────┘
│                     │
│                 Wall Outlet
└─────────────────────────────────────────────────────────────┘
```

## Detailed Wiring Connections

### Connection 1: GPS to Raspberry Pi (UART)

```
SparkFun GPS-RTK2          Raspberry Pi 4 GPIO
─────────────────          ──────────────────────
3.3V (or 5V)    ────────►  Pin 1  (3.3V Power)
GND             ────────►  Pin 6  (Ground)
TX  (Output)    ────────►  Pin 10 (GPIO 15 - RXD)
RX  (Input)     ────────►  Pin 8  (GPIO 14 - TXD)

Note: TX on GPS goes to RX on Pi (crossover connection)
```

**Alternative: GPS via USB**
```
SparkFun GPS-RTK2          Raspberry Pi 4
─────────────────          ─────────────
USB-C Port      ────────►  USB Port (any)

Device will appear as: /dev/ttyACM0
```

### Connection 2: LiDAR to Raspberry Pi (Ethernet)

```
Unitree L2 LiDAR          Raspberry Pi 4
────────────────          ─────────────
Ethernet Port   ────────►  Ethernet Port

IP Configuration:
  LiDAR:  192.168.1.62
  Pi:     192.168.1.2
  Subnet: 255.255.255.0
```

**Ethernet Cable:** Cat5e or Cat6 (any length up to 100m)

### Connection 3: Power Supply

```
Power Supply                  Components
────────────────              ───────────────

12V 2A Output     ─────────►  Unitree L2 LiDAR
  (DC barrel jack)              12V DC Input

5V 3A Output      ─────────►  Raspberry Pi 4
  (USB-C cable)                 USB-C Power Input

AC Input          ◄─────────  Wall Outlet
  (110-240V)                    (household power)
```

## Power Supply Options

### Option 1: Dual Output Power Supply (Recommended)

**Product:** Mean Well RD-65A (or similar)
- **12V Output**: 2A (for LiDAR)
- **5V Output**: 3A (for Raspberry Pi)
- **Total**: 65W
- **Cost**: ~$40-60
- **Benefit**: Single power adapter for entire system

**Where to buy:**
- Amazon: Search "dual voltage power supply 12V 5V"
- DigiKey/Mouser: Mean Well RD-65A

### Option 2: Separate Power Supplies

**For LiDAR:**
- 12V 2A DC adapter with barrel jack
- Must match Unitree L2 connector specs
- Cost: ~$15-25

**For Raspberry Pi:**
- Official Raspberry Pi 5V 3A USB-C adapter
- Cost: ~$8
- Link: https://www.raspberrypi.com/products/type-c-power-supply/

### Option 3: Battery Power (Portable)

**For mobile outdoor scanning:**

```
Battery Pack         Components
────────────         ──────────

12V LiPo/Li-ion  ───►  Unitree L2 LiDAR
  (3S or 4S)            (via voltage regulator if needed)
  ~5000mAh

USB Power Bank   ───►  Raspberry Pi 4
  5V 3A output          (via USB-C)
  ~20000mAh

Total runtime: 3-5 hours
```

**Recommended battery:**
- Talentcell 12V 6000mAh Battery Pack (~$40)
  - Has 12V output + 5V USB output
  - Can power both devices from one battery!

## Network Configuration

### Raspberry Pi Ethernet Setup

Create static IP configuration:

```bash
# Edit network config
sudo nano /etc/dhcpcd.conf

# Add at end:
interface eth0
static ip_address=192.168.1.2/24
static routers=192.168.1.1
static domain_name_servers=8.8.8.8

# Save and reboot
sudo reboot
```

### Test LiDAR Connection

```bash
# Ping the LiDAR
ping 192.168.1.62

# Should see responses:
# 64 bytes from 192.168.1.62: icmp_seq=1 ttl=64 time=0.5 ms
```

### WiFi for NTRIP (Optional)

For RTK corrections via internet while using Ethernet for LiDAR:

```bash
# Configure WiFi
sudo raspi-config
# Select: System Options > Wireless LAN
# Enter SSID and password

# Now you have:
# - eth0: Connected to LiDAR (192.168.1.2)
# - wlan0: Connected to internet (for NTRIP)
```

## GPS Antenna Placement

### For Best GPS Performance:

```
                        ☁ ☁ ☁ Clear Sky View ☁ ☁ ☁
                               │  │  │
                    GPS Satellites (visible)
                               │  │  │
                               ▼  ▼  ▼
                         ┌─────────────┐
                         │ GNSS Antenna│  ← Place here
                         │  (on roof)  │
                         └──────┬──────┘
                                │ (cable)
                                │
                        ┌───────▼────────┐
                    ────┤   Building     │
                        │                │
                        │  GPS Module ◄──┤─── Indoors OK
                        │  (GPS-RTK2)    │     (module)
                        └────────────────┘
```

**Antenna placement rules:**
1. **Outdoors** with clear sky view (360°)
2. **Elevated** (roof, pole, high mount)
3. **Away from metal** objects that cause reflections
4. **Horizontal** placement (flat, facing up)
5. **Cable length**: Keep under 5 meters if possible

**Good locations:**
- ✓ Roof of building/vehicle
- ✓ Top of mast/pole
- ✓ Open field

**Bad locations:**
- ✗ Indoors (no signal)
- ✗ Under trees (blocked)
- ✗ Near tall buildings (multipath)
- ✗ In metal enclosure (blocked)

## Complete System Assembly

### Step-by-Step Setup:

#### 1. Power Supply Setup
```
□ Connect 12V output to Unitree L2 LiDAR
□ Connect 5V output to Raspberry Pi USB-C
□ Verify voltage with multimeter (if available)
□ Do NOT plug in wall power yet
```

#### 2. GPS Module Setup
```
□ Connect GNSS antenna to GPS-RTK2 (SMA connector)
□ Place antenna outdoors with clear sky view
□ Route antenna cable indoors to GPS module
□ Connect GPS to Raspberry Pi:
  Option A (UART):
    □ 3.3V → Pin 1
    □ GND → Pin 6
    □ TX → Pin 10
    □ RX → Pin 8
  Option B (USB):
    □ USB cable to any Pi USB port
```

#### 3. LiDAR Connection
```
□ Connect Ethernet cable between LiDAR and Pi
□ Verify cable is Cat5e or better
□ Ensure connectors click into place
```

#### 4. First Power-Up
```
□ Double-check all connections
□ Plug in power supply to wall outlet
□ Raspberry Pi should boot (LED activity)
□ LiDAR should initialize (may have LED/sound)
□ GPS should start acquiring satellites
```

#### 5. Software Configuration
```
□ SSH into Raspberry Pi: ssh pi@192.168.1.2
□ Configure static IP for eth0
□ Enable UART: sudo raspi-config
□ Test LiDAR: ping 192.168.1.62
□ Test GPS: python src/positioning/gps_rtk.py
□ Run integrated scan: python examples/gps_lidar_scan.py
```

## Satellite/NTRIP Setup for RTK

### What is NTRIP?

**NTRIP** (Networked Transport of RTCM via Internet Protocol) provides RTK correction data via internet:

```
                    Internet
                       │
    ┌──────────────────┼──────────────────┐
    │                  │                  │
    ▼                  ▼                  ▼
Base Station ────► NTRIP     Your GPS ◄──┐
(known pos)        Caster    (rover)     │
    │                  │                  │
    └─ Corrections ────┴──────────────────┘

Result: cm-level accuracy!
```

### Finding an NTRIP Service

**United States:**
1. **UNAVCO** (Free for academic/research)
   - Website: https://www.unavco.org/data/gps-gnss/real-time/
   - Coverage: Nationwide
   - Accuracy: 1-2 cm

2. **CORS Network** (NOAA)
   - Website: https://geodesy.noaa.gov/CORS/
   - Coverage: Nationwide
   - Free access

3. **State DOT Networks** (Free/Paid)
   - Example: California CSRS, Florida CORS
   - Very accurate in coverage area

**Other Countries:**
- **Canada**: Canadian Active Control System
- **Europe**: EUREF Permanent Network
- **Australia**: Geoscience Australia

### Configure NTRIP on Raspberry Pi

Install NTRIP client:

```bash
# Install str2str (RTKLIB tool)
sudo apt-get install rtklib

# Or use Python NTRIP client
pip install ntripclient
```

**Python NTRIP Configuration:**

```python
from pyubx2 import UBXReader, UBXMessage, SET
import socket
import base64

# NTRIP settings (example - get from your provider)
NTRIP_SERVER = "rtgpsout.unavco.org"
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = "P041_RTCM3"
NTRIP_USER = "your_email@example.com"
NTRIP_PASS = "your_password"

# Connect to NTRIP caster
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((NTRIP_SERVER, NTRIP_PORT))

# Send NTRIP request
auth = base64.b64encode(f"{NTRIP_USER}:{NTRIP_PASS}".encode()).decode()
request = (
    f"GET /{NTRIP_MOUNTPOINT} HTTP/1.0\r\n"
    f"User-Agent: NTRIP PyClient\r\n"
    f"Authorization: Basic {auth}\r\n"
    f"\r\n"
)
sock.send(request.encode())

# Receive corrections and forward to GPS
while True:
    data = sock.recv(1024)
    # Send to GPS module via serial
    gps_serial.write(data)
```

## System Specifications

### Complete System:

| Component | Power | Current | Notes |
|-----------|-------|---------|-------|
| Unitree L2 LiDAR | 12V | 1-2A | 12-24W |
| Raspberry Pi 4 | 5V | 2-3A | 10-15W |
| GPS-RTK2 | 3.3V | 100mA | From Pi GPIO |
| GNSS Antenna | 3.3V | 50mA | From GPS module |
| **Total** | - | - | **~30-40W** |

### Data Rates:

- **LiDAR**: ~10 MB/s over Ethernet
- **GPS**: 9600-115200 baud over UART (~10 KB/s)
- **NTRIP**: ~1 KB/s over WiFi/Ethernet

### Processing Load:

- **CPU**: 40-60% (Raspberry Pi 4)
- **Memory**: 500MB-1GB
- **Storage**: 1GB per 10 minutes of scanning

## Troubleshooting

### LiDAR Not Connecting
```
Problem: Cannot ping 192.168.1.62

Solutions:
□ Check Ethernet cable is connected
□ Verify Pi IP is 192.168.1.2: ip addr show eth0
□ Check LiDAR has power (LED on)
□ Try different Ethernet cable
□ Check firewall: sudo ufw status
```

### GPS No Fix
```
Problem: GPS shows 0 satellites

Solutions:
□ Check antenna is OUTDOORS with sky view
□ Verify antenna cable is connected (SMA tight)
□ Wait 5-10 minutes (cold start takes time)
□ Check GPS has power: LEDs blinking
□ Test with: python src/positioning/gps_rtk.py
```

### No RTK Correction
```
Problem: GPS fix but no RTK

Solutions:
□ Verify internet connection (WiFi)
□ Check NTRIP credentials are correct
□ Confirm you're within 50km of base station
□ Wait 5-10 minutes for RTK convergence
□ Check GPS-RTK2 firmware is updated
```

### Power Issues
```
Problem: System reboots or unstable

Solutions:
□ Use proper 5V 3A power supply for Pi
□ Check 12V supply has enough current (2A+)
□ Verify all connections are secure
□ Try separate power supplies if using shared
□ Check for voltage drop with multimeter
```

## Cost Breakdown

| Item | Cost | Required? |
|------|------|-----------|
| Unitree L2 LiDAR | Owned | ✓ Yes |
| Raspberry Pi 4 (4GB) | $55 | ✓ Yes |
| SparkFun GPS-RTK2 | $275 | ○ Outdoor only |
| GNSS Antenna (L1/L2) | $70 | ○ With GPS |
| Dual power supply | $50 | ✓ Yes |
| Ethernet cable | $10 | ✓ Yes |
| Cables/connectors | $20 | ✓ Yes |
| **Total (with GPS)** | **~$480** | |
| **Total (without GPS)** | **~$135** | Indoor only |

## Next Steps

1. **Order hardware** based on your needs (GPS or no GPS)
2. **Assemble system** following wiring diagram
3. **Configure Raspberry Pi** network and UART
4. **Test each component** individually
5. **Run integrated scan** outdoors (with GPS) or indoors (KISS-ICP only)

The complete integration guide is ready for you to set up the system!