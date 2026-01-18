# Why GPS Doesn't Work Indoors - Technical Explanation

## The Physics

### GPS Signal Strength

GPS satellites transmit at **~50 watts** from 20,200 km altitude. By the time signals reach Earth:

```
Satellite (20,200 km altitude)
    │
    │  Signal: 50 watts
    │
    ▼
Earth Surface
    Signal received: -130 dBm (0.0000000001 watts)
```

**That's incredibly weak!** For comparison:
- WiFi router (5m away): -40 dBm (100,000,000x stronger)
- Cell phone tower: -80 dBm (100,000x stronger)
- GPS satellite: -130 dBm (baseline)

### What Happens Indoors

```
Outside (Clear Sky)          Inside Room
┌─────────────┐             ┌─────────────┐
│ Satellite 1 │             │ Satellite 1 │
└──────┬──────┘             └──────┬──────┘
       │                           │
    -130 dBm                       │
       │                           ▼
       ▼                     ╔═══════════════╗
    Antenna                  ║  ROOF/WALLS   ║  -30 to -40 dB loss
       │                     ╚═══════════════╝
    8-12 sats                      │
    Strong fix                  -160 to -170 dBm
                                   ▼
                                Antenna
                                   │
                               0-2 sats
                               No fix!
```

## Material Attenuation

### Signal Loss Through Building Materials

| Material | Thickness | Signal Loss | GPS Impact |
|----------|-----------|-------------|------------|
| **Air** | Any | 0 dB | ✓ Perfect |
| **Window (regular glass)** | 6mm | 3-5 dB | ✓ Minor loss |
| **Window (Low-E coated)** | 6mm | 20-30 dB | ✗ Heavy loss |
| **Wood roof + shingles** | 30cm | 20-30 dB | ✗ Major block |
| **Concrete floor/roof** | 15cm | 30-40 dB | ✗ Nearly complete |
| **Drywall** | 12mm | 10-15 dB | ⚠ Significant |
| **Brick wall** | 20cm | 25-35 dB | ✗ Heavy block |
| **Metal roof** | Any | 40-60 dB | ✗ Complete block |

### The Math

GPS requires **minimum -155 dBm** to decode signals.

**Outdoor:**
```
Signal arrives:    -130 dBm
Noise floor:       -155 dBm
Margin:            25 dB ✓ Good signal
```

**Indoor (with roof + wall):**
```
Signal arrives:    -130 dBm
Roof loss:         -30 dB
Wall loss:         -15 dB
Final signal:      -175 dBm
Noise floor:       -155 dBm
Margin:            -20 dB ✗ Below noise floor!
```

## Why You Need 4+ Satellites

GPS position calculation requires **trilateration**:

```
Need 4+ satellites to solve for:
  1. X position (longitude)
  2. Y position (latitude)
  3. Z position (altitude)
  4. Time correction (receiver clock error)

Indoors:
  Typical satellites visible: 0-2
  Minimum required: 4
  Result: Cannot compute position
```

## Multipath Errors Indoors

Even if you get a fix near a window, it's unreliable:

```
Outdoor (Direct Path):        Indoor (Multipath):

Satellite                     Satellite
    │                             │
    │ Direct                      │
    │ 20,200 km                   │
    ▼                             │
 Antenna                          │
                                  │
 Position: ±2m                    │
                                  ▼
                            Building/Window
                                  │
                                  │ Reflected
                                  │ 20,250 km (longer!)
                                  ▼
                               Antenna

                            Position: ±50m (ERROR!)
```

The reflected signal traveled an extra 50m, causing 50m position error!

## Real-World Indoor GPS Performance

### Typical Indoor Results:

| Location | Satellites | Fix Rate | Accuracy | Usable? |
|----------|-----------|----------|----------|---------|
| **Deep indoor** | 0-1 | 0% | N/A | ✗ No |
| **First floor, center** | 1-2 | 5-20% | 50-200m | ✗ No |
| **Near window** | 2-4 | 30-60% | 20-100m | ✗ No |
| **Skylight/atrium** | 3-6 | 60-80% | 10-50m | ⚠ Poor |
| **Open outdoor** | 8-12 | 95%+ | 2-5m | ✓ Yes |
| **Outdoor + RTK** | 8-12 | 95%+ | 0.01-0.02m | ✓✓ Excellent |

### Your Room Scanning (Indoors):
- Expected satellites: **0-2**
- Expected fix rate: **<10%**
- Expected accuracy: **50-200 meters** (when fix obtained)
- **Conclusion: GPS WILL NOT WORK**

## Alternative Technologies for Indoor Positioning

Since GPS doesn't work indoors, here are alternatives:

### 1. UWB (Ultra-Wideband) ⭐ Best for Indoor

**How it works:**
- Uses radio time-of-flight measurement
- 4+ anchors installed in room at known positions
- Tag on sensor measures distance to each anchor
- Triangulates position

**Performance:**
- ✓ Works indoors
- ✓ Accuracy: 10-30 cm
- ✓ Update rate: 10-100 Hz
- ✓ No satellite needed

**Cost:** $400 (Pozyx or Decawave kit)

**Why it works indoors:**
- Uses much higher frequency (6.5 GHz)
- Much stronger signal (transmitted locally, not from space)
- Short distances (meters, not 20,000 km)

### 2. WiFi RTT (Round-Trip Time)

**How it works:**
- Measures signal travel time to WiFi access points
- Works with WiFi 6E / 802.11mc

**Performance:**
- ✓ Works indoors
- ⚠ Accuracy: 1-3 meters
- ✓ Uses existing WiFi infrastructure

**Why it works indoors:**
- Strong local signals
- Multiple access points
- But: Less accurate than UWB

### 3. Visual-Inertial Odometry (Camera + IMU)

**How it works:**
- Camera tracks visual features
- IMU measures rotation and acceleration
- Fuses both for position estimate

**Performance:**
- ✓ Works indoors and outdoors
- ✓ No infrastructure needed
- ⚠ Accumulates drift over time

**Cost:** $30-300 (camera + IMU)

### 4. LiDAR Odometry (What You're Using!)

**How it works:**
- KISS-ICP matches consecutive point clouds
- Estimates movement from frame to frame

**Performance:**
- ✓ Works everywhere
- ✓ No additional hardware
- ⚠ Accumulates drift
- ⚠ Requires slow, careful movement

**Cost:** Free (software only)

## Comparison Table

| Technology | Indoor | Outdoor | Accuracy | Setup | Cost |
|------------|--------|---------|----------|-------|------|
| **GPS (standard)** | ✗ No | ✓ Yes | 2-5m | None | $30 |
| **GPS RTK** | ✗ No | ✓ Yes | 1-2cm | NTRIP | $350 |
| **UWB** | ✓ Yes | ⚠ Limited | 10-30cm | Anchors | $400 |
| **WiFi RTT** | ✓ Yes | ✗ No | 1-3m | APs | $0 |
| **Visual-Inertial** | ✓ Yes | ✓ Yes | Drift | None | $50 |
| **LiDAR Odom (KISS-ICP)** | ✓ Yes | ✓ Yes | Drift | None | $0 |

## Why SparkFun GPS-RTK2 Still Won't Work Indoors

Even though the ZED-F9P is a high-end GPS receiver with:
- Multi-band (L1 + L2) reception
- Advanced signal processing
- Better sensitivity than consumer GPS

**It STILL cannot work indoors because:**

1. **Physics is physics**: Roof blocks signals regardless of receiver quality
2. **L1 + L2 both blocked**: Multiple frequencies don't help if all are blocked
3. **Better sensitivity ≠ works indoors**: Can't decode signals below noise floor
4. **No satellites = no position**: Even best receiver needs 4+ satellites

The ZED-F9P is for **outdoor** high-precision work, not indoor.

## Test It Yourself

Run the indoor GPS test:

```bash
python examples/test_gps_indoor.py
```

This will show you actual satellite counts and fix rates in your location.

## Recommendation for Your LiDAR System

### Indoor Room Scanning (Your Current Work):
1. ✓ **KISS-ICP** - Move VERY slowly (10cm per 2-3 seconds)
2. ✓ **UWB** (optional) - If you need better accuracy ($400)
3. ✗ **GPS** - Will not work at all

### Outdoor Trail/Area Mapping (Future):
1. ✓ **GPS RTK** - SparkFun GPS-RTK2 + NTRIP correction
2. ✓ **KISS-ICP** - Fills in details between GPS updates
3. ✓ **Fusion** - Combine both for best results

## Summary

**GPS doesn't work indoors because:**
- Signals too weak (-130 dBm)
- Blocked by roof/walls (-30 to -40 dB loss)
- Final signal below noise floor
- Not enough satellites visible (0-2 vs 4+ required)
- Physics limitation, not technology limitation

**For indoor work:** Use UWB positioning or KISS-ICP with slow movement

**For outdoor work:** Use GPS RTK for cm-level accuracy

The hardware you're looking at (SparkFun GPS-RTK2) is **perfect for outdoor** mapping but **will not work** for indoor room scanning.