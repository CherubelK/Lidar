"""
GPS RTK Module Integration for SparkFun GPS-RTK2 (ZED-F9P)
Provides centimeter-level positioning for outdoor LiDAR scanning
"""
import serial
import pynmea2
import logging
from typing import Optional, Tuple, Dict
from datetime import datetime
import numpy as np

logger = logging.getLogger(__name__)


class GPSRTK:
    """
    Interface for SparkFun GPS-RTK2 Board (ZED-F9P) with RTK correction.

    Provides cm-level accuracy when connected to NTRIP correction service.
    """

    def __init__(self, port: str = '/dev/serial0', baudrate: int = 38400):
        """
        Initialize GPS RTK module.

        Args:
            port: Serial port (default: /dev/serial0 for Raspberry Pi UART)
                  Use /dev/ttyACM0 or /dev/ttyUSB0 for USB connection
            baudrate: Baud rate (ZED-F9P default: 38400)
        """
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            logger.info(f"GPS RTK connected on {port} at {baudrate} baud")
        except serial.SerialException as e:
            logger.error(f"Failed to connect to GPS: {e}")
            raise

        self.current_position = None
        self.fix_quality = 0
        self.num_satellites = 0
        self.hdop = 99.9  # Horizontal Dilution of Precision
        self.rtk_age = None
        self.rtk_ratio = None

    def get_position(self) -> Optional[Dict]:
        """
        Get current GPS position with RTK correction.

        Returns:
            Dictionary with GPS data or None if no valid fix:
            {
                'latitude': float (degrees),
                'longitude': float (degrees),
                'altitude': float (meters above MSL),
                'fix_quality': int (0=no fix, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float),
                'num_satellites': int,
                'hdop': float (horizontal dilution of precision),
                'timestamp': datetime,
                'rtk_age': float (age of RTK correction in seconds),
                'rtk_ratio': float (RTK solution quality ratio)
            }
        """
        try:
            line = self.serial.readline().decode('ascii', errors='ignore')

            # Parse GGA message (Global Positioning System Fix Data)
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)

                self.fix_quality = int(msg.gps_qual)
                self.num_satellites = int(msg.num_sats) if msg.num_sats else 0
                self.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 99.9

                if self.fix_quality > 0:
                    self.current_position = {
                        'latitude': msg.latitude,
                        'longitude': msg.longitude,
                        'altitude': msg.altitude,
                        'fix_quality': self.fix_quality,
                        'num_satellites': self.num_satellites,
                        'hdop': self.hdop,
                        'timestamp': datetime.utcnow(),
                        'rtk_age': self.rtk_age,
                        'rtk_ratio': self.rtk_ratio
                    }
                    return self.current_position

            # Parse RMC message (Recommended Minimum Specific GNSS Data)
            elif line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)
                # RMC provides speed and heading
                if hasattr(msg, 'spd_over_grnd') and self.current_position:
                    self.current_position['speed_knots'] = msg.spd_over_grnd
                    self.current_position['heading'] = msg.true_course

            # Parse GST message (GNSS Pseudorange Error Statistics)
            elif line.startswith('$GNGST'):
                # GST provides accuracy estimates
                pass

        except (pynmea2.ParseError, UnicodeDecodeError) as e:
            logger.debug(f"GPS parse error: {e}")
        except Exception as e:
            logger.error(f"GPS error: {e}")

        return None

    def get_fix_quality_string(self) -> str:
        """Get human-readable fix quality description."""
        fix_types = {
            0: "No Fix",
            1: "GPS Fix",
            2: "DGPS Fix",
            3: "PPS Fix",
            4: "RTK Fixed",
            5: "RTK Float",
            6: "Estimated",
            7: "Manual",
            8: "Simulation"
        }
        return fix_types.get(self.fix_quality, "Unknown")

    def has_rtk_fix(self) -> bool:
        """Check if we have RTK fixed solution (cm-level accuracy)."""
        return self.fix_quality == 4

    def has_rtk_float(self) -> bool:
        """Check if we have RTK float solution (dm-level accuracy)."""
        return self.fix_quality == 5

    def has_valid_fix(self) -> bool:
        """Check if we have any valid GPS fix."""
        return self.fix_quality >= 1

    def wait_for_fix(self, timeout: int = 60, rtk_required: bool = False) -> bool:
        """
        Wait for GPS fix (optionally RTK fix).

        Args:
            timeout: Maximum wait time in seconds
            rtk_required: If True, wait for RTK fix (quality 4 or 5)

        Returns:
            True if fix acquired, False if timeout
        """
        import time
        start_time = time.time()

        print(f"Waiting for {'RTK' if rtk_required else 'GPS'} fix...")

        while (time.time() - start_time) < timeout:
            position = self.get_position()

            if position:
                if rtk_required:
                    if self.has_rtk_fix() or self.has_rtk_float():
                        print(f"RTK fix acquired! Quality: {self.get_fix_quality_string()}")
                        return True
                else:
                    if self.has_valid_fix():
                        print(f"GPS fix acquired! Quality: {self.get_fix_quality_string()}")
                        return True

            # Print status every 5 seconds
            if int(time.time() - start_time) % 5 == 0:
                print(f"  Satellites: {self.num_satellites}, Quality: {self.get_fix_quality_string()}")

            time.sleep(0.1)

        print(f"Timeout waiting for {'RTK' if rtk_required else 'GPS'} fix")
        return False

    def get_utm_coordinates(self) -> Optional[Tuple[float, float, float, int, str]]:
        """
        Convert GPS coordinates to UTM (Universal Transverse Mercator).

        Returns:
            Tuple of (easting, northing, altitude, zone_number, zone_letter) or None
        """
        if not self.current_position:
            return None

        try:
            import utm
            easting, northing, zone_number, zone_letter = utm.from_latlon(
                self.current_position['latitude'],
                self.current_position['longitude']
            )
            return (easting, northing, self.current_position['altitude'],
                    zone_number, zone_letter)
        except ImportError:
            logger.error("utm library not installed. Run: pip install utm")
            return None

    def get_local_coordinates(self, origin_lat: float, origin_lon: float) -> Optional[Tuple[float, float, float]]:
        """
        Get local coordinates relative to an origin point.
        Useful for LiDAR scanning with local coordinate frame.

        Args:
            origin_lat: Origin latitude (degrees)
            origin_lon: Origin longitude (degrees)

        Returns:
            Tuple of (x, y, z) in meters relative to origin, or None
        """
        if not self.current_position:
            return None

        # Use simple approximation for small distances (<100km)
        # For better accuracy, use UTM coordinates

        lat = self.current_position['latitude']
        lon = self.current_position['longitude']
        alt = self.current_position['altitude']

        # Approximate conversion (good for small distances)
        lat_diff = lat - origin_lat
        lon_diff = lon - origin_lon

        # Meters per degree at equator
        meters_per_lat = 111320.0  # meters per degree latitude
        meters_per_lon = 111320.0 * np.cos(np.radians(origin_lat))  # varies with latitude

        x = lon_diff * meters_per_lon
        y = lat_diff * meters_per_lat
        z = alt  # altitude already in meters

        return (x, y, z)

    def close(self):
        """Close serial connection."""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            logger.info("GPS RTK connection closed")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# Example usage
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    # Connect to GPS (adjust port as needed)
    with GPSRTK(port='/dev/serial0') as gps:

        # Wait for GPS fix
        if gps.wait_for_fix(timeout=120, rtk_required=False):

            # Get position
            position = gps.get_position()

            if position:
                print("\nGPS Position:")
                print(f"  Latitude: {position['latitude']:.8f}°")
                print(f"  Longitude: {position['longitude']:.8f}°")
                print(f"  Altitude: {position['altitude']:.2f}m")
                print(f"  Fix Quality: {gps.get_fix_quality_string()}")
                print(f"  Satellites: {position['num_satellites']}")
                print(f"  HDOP: {position['hdop']:.2f}")

                if gps.has_rtk_fix():
                    print("\n✓ RTK Fixed - Centimeter accuracy!")
                elif gps.has_rtk_float():
                    print("\n✓ RTK Float - Decimeter accuracy")

                # Get UTM coordinates
                utm_coords = gps.get_utm_coordinates()
                if utm_coords:
                    print(f"\nUTM Coordinates:")
                    print(f"  Easting: {utm_coords[0]:.2f}m")
                    print(f"  Northing: {utm_coords[1]:.2f}m")
                    print(f"  Zone: {utm_coords[3]}{utm_coords[4]}")