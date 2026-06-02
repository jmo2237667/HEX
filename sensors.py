# sensors.py
# ============================================================================
# Onboard Sensor Drivers -- HC-SR04 Ultrasonic + MPU6050 IMU
# ============================================================================

import time
import math
from gpiozero import DistanceSensor
from mpu6050 import mpu6050


class UltrasonicSensor:
    """
    Non-blocking HC-SR04 distance sensor via gpiozero.

    gpiozero's DistanceSensor uses an internal background pin-watching
    thread to cache the latest echo timing.  Reading the .distance
    property is instantaneous and never blocks the main loop.
    """

    def __init__(self, trigger_pin=17, echo_pin=27):
        # gpiozero uses BCM numbering by default
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin,
                                     max_distance=4.0)

    def get_distance_cm(self):
        """Returns distance in centimetres (gpiozero reports metres)."""
        return self.sensor.distance * 100.0


class Gyroscope:
    """
    MPU6050 6-axis IMU with high-pass / low-pass Complementary Filter
    orientation fusion.

    Fuses the accelerometer (low-frequency, drift-free but noisy) with
    the gyroscope (high-frequency, smooth but drifts over time) using
    a tunable alpha coefficient.

        filtered_angle = alpha * (prev_angle + gyro_rate * dt)
                       + (1 - alpha) * accel_angle

    Alpha = 0.96 trusts the gyroscope for 96 % of the estimate and
    corrects long-term drift with the accelerometer at 4 %.
    """

    ALPHA = 0.96

    def __init__(self, address=0x68):
        self.connected = False
        self._pitch = 0.0
        self._roll  = 0.0
        self._last_time = time.monotonic()

        try:
            self.sensor = mpu6050(address)
            self.connected = True
            print(f"[IMU] MPU6050 connected at {hex(address)}")
        except Exception as e:
            print(f"[IMU] MPU6050 not found at {hex(address)}: {e}")
            print("[IMU] Running with mock orientation data (0.0, 0.0)")

    def get_orientation(self):
        """
        Returns (pitch, roll) in degrees using complementary filter fusion.

        Called only from the main thread -- no cross-thread I2C contention.
        """
        if not self.connected:
            return (0.0, 0.0)

        try:
            now = time.monotonic()
            dt = now - self._last_time
            self._last_time = now

            # Clamp dt to prevent wild spikes after long pauses
            dt = min(dt, 0.1)

            accel = self.sensor.get_accel_data()
            gyro  = self.sensor.get_gyro_data()

            ax = accel['x']
            ay = accel['y']
            az = accel['z']

            # Accelerometer-derived angles (noisy but drift-free, low-pass)
            accel_pitch = math.degrees(
                math.atan2(ay, math.sqrt(ax * ax + az * az))
            )
            accel_roll = math.degrees(math.atan2(-ax, az))

            # Gyroscope rates in deg/s (smooth but drifts, high-pass)
            gyro_pitch_rate = gyro['y']
            gyro_roll_rate  = gyro['x']

            # Complementary filter fusion
            self._pitch = (self.ALPHA * (self._pitch + gyro_pitch_rate * dt) +
                           (1.0 - self.ALPHA) * accel_pitch)
            self._roll  = (self.ALPHA * (self._roll + gyro_roll_rate * dt) +
                           (1.0 - self.ALPHA) * accel_roll)

            return (self._pitch, self._roll)

        except Exception:
            # Return last known good values on transient I2C failure
            return (self._pitch, self._roll)
