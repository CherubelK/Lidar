"""
Test script to demonstrate why GPS doesn't work indoors
This will show you actual GPS performance in different locations
"""
import sys
from pathlib import Path
import time
from datetime import datetime

sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from src.positioning.gps_rtk import GPSRTK
except ImportError:
    print("ERROR: GPS module not available")
    print("This is a demonstration script showing what WOULD happen")
    print("if you tried to use GPS indoors.")
    sys.exit(1)

print("="*70)
print("GPS INDOOR vs OUTDOOR TEST")
print("="*70)
print()
print("This test will show you why GPS doesn't work indoors.")
print()
print("Test procedure:")
print("  1. Place GPS antenna in current location")
print("  2. Wait 2 minutes")
print("  3. Record results")
print()

gps_port = input("GPS serial port (default: /dev/serial0): ").strip() or "/dev/serial0"
test_location = input("Where is the antenna? (indoor/window/outdoor): ").strip().lower()

print()
print(f"Testing GPS at: {test_location}")
print(f"Port: {gps_port}")
print()

try:
    gps = GPSRTK(port=gps_port)
except Exception as e:
    print(f"Could not connect to GPS: {e}")
    sys.exit(1)

print("Monitoring GPS for 120 seconds...")
print()
print("Time | Satellites | Fix Type | HDOP | Latitude | Longitude")
print("-" * 70)

start_time = time.time()
fix_count = 0
total_readings = 0

satellites_seen = []
fix_qualities = []

while (time.time() - start_time) < 120:
    position = gps.get_position()
    total_readings += 1

    elapsed = int(time.time() - start_time)

    if position:
        fix_count += 1
        satellites_seen.append(position['num_satellites'])
        fix_qualities.append(gps.get_fix_quality_string())

        print(f"{elapsed:3d}s | {position['num_satellites']:10d} | "
              f"{gps.get_fix_quality_string():12s} | "
              f"{position['hdop']:4.1f} | "
              f"{position['latitude']:11.6f} | "
              f"{position['longitude']:12.6f}")
    else:
        satellites_seen.append(gps.num_satellites)
        print(f"{elapsed:3d}s | {gps.num_satellites:10d} | "
              f"{'No Fix':12s} | "
              f"{'--':>4s} | "
              f"{'--':>11s} | "
              f"{'--':>12s}")

    time.sleep(2)

gps.close()

print()
print("="*70)
print("RESULTS")
print("="*70)
print()
print(f"Location: {test_location}")
print(f"Duration: 120 seconds")
print(f"Total readings: {total_readings}")
print(f"Valid fixes: {fix_count}")
print(f"Fix rate: {fix_count/total_readings*100:.1f}%")
print()

if satellites_seen:
    import numpy as np
    print(f"Satellites:")
    print(f"  Average: {np.mean(satellites_seen):.1f}")
    print(f"  Maximum: {max(satellites_seen)}")
    print(f"  Minimum: {min(satellites_seen)}")
    print()

print("="*70)
print("ANALYSIS")
print("="*70)
print()

if test_location == "indoor":
    if fix_count < total_readings * 0.1:  # Less than 10% fix rate
        print("✗ GPS DOES NOT WORK INDOORS")
        print()
        print("Why:")
        print("  - Roof blocks satellite signals")
        print("  - Walls cause signal attenuation")
        print("  - Not enough satellites visible (need 4+)")
        print("  - Signal too weak to decode")
        print()
        print("Expected results INDOORS:")
        print("  - 0-3 satellites seen")
        print("  - 0-20% fix rate")
        print("  - 50+ meter errors when fix is obtained")
        print("  - No RTK possible")
    else:
        print("⚠ LIMITED GPS INDOORS")
        print()
        print("You got some fixes, but this is NOT reliable:")
        print("  - Probably near a window")
        print("  - Signals are reflected/multipath")
        print("  - Position accuracy: 20-100 meters")
        print("  - Not suitable for mapping!")

elif test_location == "window":
    if fix_count > total_readings * 0.5:
        print("⚠ PARTIAL GPS AT WINDOW")
        print()
        print("Why it partially works:")
        print("  - Direct view to some satellites through glass")
        print("  - Signal reflections from outside")
        print()
        print("Problems:")
        print("  - Multipath errors (10-50 meters)")
        print("  - Intermittent fix")
        print("  - No RTK correction")
        print("  - Not suitable for precision mapping")
    else:
        print("✗ GPS DOES NOT WORK WELL AT WINDOW")
        print()
        print("Even near a window, GPS is unreliable indoors.")

elif test_location == "outdoor":
    if fix_count > total_readings * 0.8:
        print("✓ GPS WORKS OUTDOORS!")
        print()
        print("Expected performance:")
        print("  - 8-12 satellites visible")
        print("  - >90% fix rate")
        print("  - 2-5 meter accuracy (standard GPS)")
        print("  - 1-2 cm accuracy (with RTK correction)")
        print()
        print("This is where you should use GPS for mapping!")
    else:
        print("⚠ OBSTRUCTED OUTDOOR LOCATION")
        print()
        print("You're outdoors but something is blocking:")
        print("  - Trees overhead")
        print("  - Buildings nearby")
        print("  - Move to open area for best results")

print()
print("="*70)
print("RECOMMENDATION")
print("="*70)
print()
print("For your LiDAR scanning:")
print()
print("INDOOR MAPPING:")
print("  ✗ Do NOT use GPS (it won't work)")
print("  ✓ Use KISS-ICP alone (move SLOWLY)")
print("  ✓ Or add UWB positioning ($400, 10-30cm accuracy)")
print()
print("OUTDOOR MAPPING:")
print("  ✓ Use GPS RTK (SparkFun GPS-RTK2 + antenna)")
print("  ✓ Combine with KISS-ICP for best results")
print("  ✓ Get RTK correction for cm-level accuracy")
print()
print("="*70)