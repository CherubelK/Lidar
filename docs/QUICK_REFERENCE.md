# Quick Reference Card - Complete LiDAR System

## System Components

```
┌─────────────────────────────────────────────────────────┐
│  Unitree L2 ◄──Ethernet──► Raspberry Pi ◄──UART──► GPS │
│    (LiDAR)     192.168.1.x      (CPU)      /dev/ttyX    │
│      12V                          5V              3.3V  │
└─────────────────────────────────────────────────────────┘
```

## Pin Connections

### GPS to Raspberry Pi (UART)
```
GPS-RTK2    →    Raspberry Pi GPIO
────────         ─────────────────
3.3V        →    Pin 1  (3.3V)
GND         →    Pin 6  (GND)
TX          →    Pin 10 (RXD/GPIO15)
RX          →    Pin 8  (TXD/GPIO14)
```

## IP Addresses

```
LiDAR:  192.168.1.62
Pi:     192.168.1.2
```

## Power Requirements

```
Component          Voltage    Current    Power
─────────────────────────────────────────────
Unitree L2 LiDAR   12V DC     1-2A       12-24W
Raspberry Pi 4     5V DC      2-3A       10-15W
GPS-RTK2          3.3V DC     100mA      0.5W
─────────────────────────────────────────────
TOTAL                                    ~30-40W
```

## Common Commands

### Test LiDAR Connection
```bash
ping 192.168.1.62
```

### Test GPS
```bash
python src/positioning/gps_rtk.py
```

### Indoor Scan (KISS-ICP only)
```bash
python examples/kiss_icp_scan.py
```

### Outdoor Scan (GPS + LiDAR)
```bash
python examples/gps_lidar_scan.py
```

### View Results
```
http://localhost:8000/viewer.html
```

## Scanning Tips

### Indoor (No GPS)
- ✓ Move VERY slowly (10cm per 2-3 seconds)
- ✓ Walk perimeter + internal grid pattern
- ✓ Return to start for loop closure
- ✗ Don't use GPS (won't work)

### Outdoor (With GPS)
- ✓ Place antenna outdoors with sky view
- ✓ Wait for RTK fix (if using NTRIP)
- ✓ Walk steady pace (~1 m/s)
- ✓ GPS updates every 1 second
- ✓ LiDAR fills in details at 5 Hz

## Troubleshooting Quick Checks

```
Problem              Quick Fix
────────────────────────────────────────────────
LiDAR not found      Check: ping 192.168.1.62
GPS no satellites    Move antenna OUTDOORS
Pi won't boot        Check 5V power supply (3A)
Permission denied    sudo usermod -a -G dialout $USER
```

## File Locations

```
Scripts:        examples/
GPS Module:     src/positioning/gps_rtk.py
Scan Data:      data/kiss_icp/ or data/gps_lidar/
Web Viewer:     web/viewer.html
Models:         web/models/
Docs:           docs/
```

## GPS Fix Quality

```
Quality      Name         Accuracy    Use for Mapping?
─────────────────────────────────────────────────────
0            No Fix       N/A         ✗ No
1            GPS          2-5m        ⚠ Poor
2            DGPS         1-2m        ⚠ Acceptable
4            RTK Fixed    1-2cm       ✓ Excellent
5            RTK Float    10-20cm     ✓ Good
```

## Antenna Placement

```
✓ GOOD                    ✗ BAD
─────────────────────────────────────
Roof/outdoors             Indoors
Clear sky view            Under trees
Elevated position         In metal box
Away from metal           Near buildings
Horizontal/flat           Any angle
```

## Quick Decision Tree

```
Where scanning?
   │
   ├─► Indoor
   │    └─► Use: KISS-ICP only
   │         Move: VERY slowly
   │         GPS: Not needed
   │
   └─► Outdoor
        └─► Use: GPS + KISS-ICP
             Move: Steady pace
             GPS: Essential
             RTK: Recommended
```

## Parts List & Costs

```
Indoor Setup (KISS-ICP only):
  □ Raspberry Pi 4 (4GB)      $55
  □ Power supply (5V 3A)      $15
  □ Ethernet cable            $10
  Total:                      $80

Outdoor Setup (GPS + LiDAR):
  □ Above items               $80
  □ SparkFun GPS-RTK2         $275
  □ GNSS Antenna (L1/L2)      $70
  □ 12V power supply          $25
  Total:                      $450
```

## Support & Documentation

```
Full Integration:   docs/SYSTEM_INTEGRATION.md
GPS Setup:          examples/README_GPS_SETUP.md
KISS-ICP Guide:     examples/README_KISS_ICP.md
Why GPS fails:      docs/WHY_GPS_FAILS_INDOORS.md
Positioning:        examples/README_POSITIONING_OPTIONS.md
```