"""
Example: Basic LiDAR Data Capture

This example demonstrates how to capture LiDAR data from the Unitree L2 sensor.
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lidar_interface import LiDARDataCapture, LiDARConfig
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def main():
    """Run a basic capture session."""

    # Configure LiDAR (adjust these values for your setup)
    config = LiDARConfig(
        ip_address="192.168.1.1",  # Update with your sensor's IP
        port=2368,
        frame_rate=10,
        range_min=0.5,
        range_max=30.0
    )

    # Create capture instance
    capture = LiDARDataCapture(
        output_dir="data/raw",
        lidar_config=config
    )

    # Start session
    print("Starting capture session...")
    if not capture.start_session(session_name="test_trail"):
        print("Failed to start session!")
        return

    try:
        # Capture for 10 seconds
        print("Capturing data for 10 seconds...")
        frames_captured = capture.capture_continuous(duration_seconds=10)

        print(f"\nCapture complete! Captured {frames_captured} frames")
        print(f"Data saved to: {capture.session_dir}")

    except KeyboardInterrupt:
        print("\nCapture interrupted by user")

    finally:
        # Clean up
        capture.end_session()
        print("Session ended")


if __name__ == "__main__":
    main()