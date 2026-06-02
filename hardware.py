# hardware.py
# ============================================================================
# PCA9685 16-Channel Servo Driver Interface
# ============================================================================

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import config


class HexapodHardware:
    """
    Manages I2C initialisation, per-channel angle commands with calibration
    offsets and safety limits, and clean electrical shutdown.
    """

    def __init__(self):
        print("[HW] Initializing I2C bus...")
        self.i2c = busio.I2C(board.SCL, board.SDA)

        print(f"[HW] Initializing PCA9685 at 0x{config.I2C_ADDRESS:02X}...")
        self.pca = PCA9685(self.i2c, address=config.I2C_ADDRESS)
        self.pca.frequency = config.PWM_FREQ

        self.servos = []
        for i in range(16):
            self.servos.append(servo.Servo(self.pca.channels[i]))

        print("[HW] PCA9685 online -- 16 servo channels ready.")

    def set_angle(self, channel, angle):
        """
        Sets the angle for a specific servo channel, applying calibration
        offsets and safety limits.
        """
        if not (0 <= channel <= 15):
            print(f"[HW] Error: Invalid channel {channel}")
            return

        # Apply calibration offset
        calibrated_angle = angle + config.CALIBRATION_OFFSETS.get(channel, 0)

        # Constrain angle to safety limits
        safe_angle = max(config.DEFAULT_MIN_ANGLE,
                         min(config.DEFAULT_MAX_ANGLE, calibrated_angle))

        # Command the servo
        self.servos[channel].angle = safe_angle

    def move_leg_neutral(self, leg_name):
        """Moves a specific leg to its neutral stance."""
        leg_info = config.LEGS.get(leg_name)
        if not leg_info:
            return

        channels = leg_info['channels']

        if 'coxa' in channels:
            self.set_angle(channels['coxa'], config.NEUTRAL_STANCE['coxa'])
        if 'femur' in channels:
            self.set_angle(channels['femur'], config.NEUTRAL_STANCE['femur'])
        if 'tibia' in channels:
            self.set_angle(channels['tibia'], config.NEUTRAL_STANCE['tibia'])

    def set_all_neutral(self):
        """Moves all legs to their neutral stance."""
        for leg_name in config.LEGS.keys():
            self.move_leg_neutral(leg_name)

    def deinit(self):
        """
        Releases PCA9685 resources safely.

        Sets all 16 channel duty cycles to 0 to electrically relax active
        servos before releasing the I2C bus, preventing servos from holding
        their last commanded position indefinitely after shutdown.
        """
        print("[HW] Setting all channel duty cycles to 0...")
        for i in range(16):
            try:
                self.pca.channels[i].duty_cycle = 0
            except Exception:
                pass

        print("[HW] Releasing PCA9685 I2C resources...")
        try:
            self.pca.deinit()
        except Exception:
            pass
