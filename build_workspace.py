#!/usr/bin/env python3
"""
build_workspace.py
============================================================================
MASTER DEPLOYMENT SCRIPT — Hexapod Advanced Autonomous Agent
============================================================================

Generates the complete production-ready codebase for an 18-servo hexapod
robot running on a Raspberry Pi 3B with PCA9685, PS5 DualSense, HC-SR04,
and MPU6050 hardware.

Run:
    python build_workspace.py

Output directory:
    ./hexapod_robot/

All files are written atomically with zero placeholders.
============================================================================
"""

import os
import sys


# ============================================================================
# Target workspace directory
# ============================================================================

WORKSPACE = "hexapod_robot"


# ############################################################################
#
#   FILE 1 / 9 :  requirements.txt
#
# ############################################################################

REQUIREMENTS_TXT = """\
Adafruit-Blinka
adafruit-circuitpython-pca9685
adafruit-circuitpython-motor
evdev
scikit-learn
numpy
flask
opencv-python-headless
gpiozero
mpu6050-raspberrypi
"""


# ############################################################################
#
#   FILE 2 / 9 :  config.py
#
# ############################################################################

CONFIG_PY = """\
# config.py
# ============================================================================
# Hexapod Robot -- Master Configuration
# Hardware: Raspberry Pi 3B + PCA9685 16-Channel PWM Driver + Elegoo Kit
# ============================================================================


# ============================================================================
# 1. PCA9685 I2C Bus Configuration
# ============================================================================

I2C_ADDRESS = 0x40          # Default PCA9685 I2C address
PWM_FREQ    = 50            # Standard servo PWM frequency (50 Hz = 20ms period)


# ============================================================================
# 2. Servo Pulse Calibration (Raw PCA9685 12-bit Tick Counts, 0-4095)
# ============================================================================
# Standard 9g analog servo operating range: 500 us - 2400 us
#
# At 50 Hz the PCA9685 period is 20 ms = 20,000 us.
# 12-bit resolution: 4096 ticks per period.
# 1 tick = 20,000 us / 4096 = 4.8828 us.
#
#    500 us  ->  tick 102   (~   0 deg, hard mechanical limit)
#   1450 us  ->  tick 297   (~  90 deg, neutral center)
#   2400 us  ->  tick 492   (~ 180 deg, hard mechanical limit)
#
# The safe window below keeps the horn well inside the physical stops
# of a standard 9g metal-gear servo to prevent stalling or gear stripping.

SERVO_MIN_PULSE_US     = 500
SERVO_MAX_PULSE_US     = 2400
SERVO_NEUTRAL_PULSE_US = 1450

SERVO_MIN_TICK     = 102    # 500 us  -> ~  0 deg
SERVO_MAX_TICK     = 492    # 2400 us -> ~180 deg
SERVO_NEUTRAL_TICK = 297    # 1450 us -> ~ 90 deg

# Software-enforced angular limits (degrees) -- extra margin vs. mechanical stops
DEFAULT_MIN_ANGLE  = 10
DEFAULT_MAX_ANGLE  = 170

# Default neutral/stance angles (degrees)
NEUTRAL_STANCE = {
    'coxa':  90,    # Hip centered horizontally
    'femur': 90,    # Thigh parallel to ground plane
    'tibia': 90,    # Shin perpendicular to thigh
}

# Per-channel calibration offsets (degrees).
# Tune each value after assembly to correct for servo horn alignment error.
CALIBRATION_OFFSETS = {ch: 0 for ch in range(16)}


# ============================================================================
# 3. Physical Leg Geometry (millimetres)
# ============================================================================
# Consumed by kinematics.py for inverse-kinematics calculations.

COXA_LENGTH  = 29.0         # Hip segment (horizontal swing link)
FEMUR_LENGTH = 40.0         # Thigh segment
TIBIA_LENGTH = 75.0         # Shin segment


# ============================================================================
# 4. Servo Channel Map -- 16 Channels (0-15)
# ============================================================================
# 3-DOF corner legs: Coxa (hip swing) + Femur + Tibia   = 12 servos
# 2-DOF middle legs: Femur + Tibia only (no coxa)       =  4 servos
#                                                  Total = 16 servos

LEGS = {
    # ---- 3-DOF Front Corner Legs ----
    'front_left':   {
        'type': '3-dof',
        'channels': {'coxa':  0, 'femur':  1, 'tibia':  2},
    },
    'front_right':  {
        'type': '3-dof',
        'channels': {'coxa':  3, 'femur':  4, 'tibia':  5},
    },

    # ---- 2-DOF Middle Legs (no coxa) ----
    'middle_left':  {
        'type': '2-dof',
        'channels': {            'femur':  6, 'tibia':  7},
    },
    'middle_right': {
        'type': '2-dof',
        'channels': {            'femur':  8, 'tibia':  9},
    },

    # ---- 3-DOF Rear Corner Legs ----
    'rear_left':    {
        'type': '3-dof',
        'channels': {'coxa': 10, 'femur': 11, 'tibia': 12},
    },
    'rear_right':   {
        'type': '3-dof',
        'channels': {'coxa': 13, 'femur': 14, 'tibia': 15},
    },
}


# ============================================================================
# 5. Flask FPV Video Stream -- Network Settings
# ============================================================================

FLASK_HOST = '0.0.0.0'     # Bind to all interfaces (accessible over WiFi)
FLASK_PORT = 5000           # Default HTTP port for the MJPEG stream


# ============================================================================
# 6. Main Loop Timing
# ============================================================================

LOOP_DELAY = 0.02           # Execution throttle (seconds) for bypass paths.
                            # Prevents CPU starvation on Pi 3B when the main
                            # loop hits a 'continue' before reaching the gait
                            # tick (which has its own internal sleep).
"""


# ############################################################################
#
#   FILE 3 / 9 :  hardware.py
#
# ############################################################################

HARDWARE_PY = """\
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
    \"\"\"
    Manages I2C initialisation, per-channel angle commands with calibration
    offsets and safety limits, and clean electrical shutdown.
    \"\"\"

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
        \"\"\"
        Sets the angle for a specific servo channel, applying calibration
        offsets and safety limits.
        \"\"\"
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
        \"\"\"Moves a specific leg to its neutral stance.\"\"\"
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
        \"\"\"Moves all legs to their neutral stance.\"\"\"
        for leg_name in config.LEGS.keys():
            self.move_leg_neutral(leg_name)

    def deinit(self):
        \"\"\"
        Releases PCA9685 resources safely.

        Sets all 16 channel duty cycles to 0 to electrically relax active
        servos before releasing the I2C bus, preventing servos from holding
        their last commanded position indefinitely after shutdown.
        \"\"\"
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
"""


# ############################################################################
#
#   FILE 4 / 9 :  sensors.py
#
# ############################################################################

SENSORS_PY = """\
# sensors.py
# ============================================================================
# Onboard Sensor Drivers -- HC-SR04 Ultrasonic + MPU6050 IMU
# ============================================================================

import time
import math
from gpiozero import DistanceSensor
from mpu6050 import mpu6050


class UltrasonicSensor:
    \"\"\"
    Non-blocking HC-SR04 distance sensor via gpiozero.

    gpiozero's DistanceSensor uses an internal background pin-watching
    thread to cache the latest echo timing.  Reading the .distance
    property is instantaneous and never blocks the main loop.
    \"\"\"

    def __init__(self, trigger_pin=17, echo_pin=27):
        # gpiozero uses BCM numbering by default
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin,
                                     max_distance=4.0)

    def get_distance_cm(self):
        \"\"\"Returns distance in centimetres (gpiozero reports metres).\"\"\"
        return self.sensor.distance * 100.0


class Gyroscope:
    \"\"\"
    MPU6050 6-axis IMU with high-pass / low-pass Complementary Filter
    orientation fusion.

    Fuses the accelerometer (low-frequency, drift-free but noisy) with
    the gyroscope (high-frequency, smooth but drifts over time) using
    a tunable alpha coefficient.

        filtered_angle = alpha * (prev_angle + gyro_rate * dt)
                       + (1 - alpha) * accel_angle

    Alpha = 0.96 trusts the gyroscope for 96 % of the estimate and
    corrects long-term drift with the accelerometer at 4 %.
    \"\"\"

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
        \"\"\"
        Returns (pitch, roll) in degrees using complementary filter fusion.

        Called only from the main thread -- no cross-thread I2C contention.
        \"\"\"
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
"""


# ############################################################################
#
#   FILE 5 / 9 :  video_stream.py
#
# ############################################################################

VIDEO_STREAM_PY = """\
# video_stream.py
# ============================================================================
# Thread-Safe Flask FPV MJPEG Video Server
# ============================================================================
#
# Architecture:
#   - A dedicated daemon thread captures frames from the USB camera and
#     writes JPEG-encoded bytes into a shared buffer protected by a Lock.
#   - The Flask MJPEG generator reads from that buffer under the same Lock.
#   - The Flask/Werkzeug server itself runs on a separate daemon thread.
#
# This design ensures the camera is touched by exactly one thread, and
# the frame buffer access is serialised via threading.Lock.
# ============================================================================

import time
import threading
import cv2
from flask import Flask, Response
import config

app = Flask(__name__)

_frame_lock = threading.Lock()
_latest_frame = None
_capture_started = False


def _camera_capture_loop():
    \"\"\"
    Background camera capture loop.  Runs on its own daemon thread.
    Grabs frames from the USB camera, encodes to JPEG, and stores
    in a thread-safe buffer that the Flask generator reads from.
    \"\"\"
    global _latest_frame

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 15)

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 70]

    while True:
        success, frame = cap.read()
        if not success:
            time.sleep(0.05)
            continue

        ret, buf = cv2.imencode('.jpg', frame, encode_params)
        if ret:
            with _frame_lock:
                _latest_frame = buf.tobytes()


def _ensure_capture_started():
    \"\"\"Start the camera capture thread if not already running.\"\"\"
    global _capture_started
    if not _capture_started:
        _capture_started = True
        t = threading.Thread(target=_camera_capture_loop, daemon=True)
        t.start()


def _generate_frames():
    \"\"\"MJPEG frame generator consumed by Flask's Response streamer.\"\"\"
    _ensure_capture_started()

    while True:
        with _frame_lock:
            frame_bytes = _latest_frame

        if frame_bytes is None:
            time.sleep(0.05)
            continue

        yield (b'--frame\\r\\n'
               b'Content-Type: image/jpeg\\r\\n\\r\\n' + frame_bytes + b'\\r\\n')

        # Cap output rate to ~30 FPS to avoid saturating the WiFi link
        time.sleep(0.033)


@app.route('/video_feed')
def video_feed():
    return Response(_generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return ("<html><body><h1>Hexapod FPV Stream</h1>"
            "<img src='/video_feed'></body></html>")


def run_flask_app():
    \"\"\"Run Flask in production-ish mode (no reloader, no debug).\"\"\"
    app.run(host=config.FLASK_HOST, port=config.FLASK_PORT,
            debug=False, use_reloader=False)


def start_stream_thread():
    \"\"\"Spawn the Flask FPV server on a daemon thread.\"\"\"
    print(f"[FPV] Starting Flask MJPEG stream on port {config.FLASK_PORT}...")
    thread = threading.Thread(target=run_flask_app, daemon=True)
    thread.start()
"""


# ############################################################################
#
#   FILE 6 / 9 :  kinematics.py
#
# ############################################################################

KINEMATICS_PY = """\
# kinematics.py
# ============================================================================
# Inverse Kinematics Engine -- 3-DOF and 2-DOF Leg Solvers
# ============================================================================
#
# Coordinate convention:
#   x  : lateral distance outward from body (positive = outward)
#   y  : longitudinal distance (positive = forward)
#   z  : depth below body plane (MUST be non-positive; negative = downward)
#
# abs(z) sign protection is enforced at the entry of every solver to
# guarantee the depth coordinate is always non-positive, regardless of
# the sign convention used by the caller.
# ============================================================================

import math
from config import COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH


def inverse_kinematics_3dof(x, y, z):
    \"\"\"
    Computes joint angles for a 3-DOF leg.

    Parameters
    ----------
    x : float   Lateral distance outward from the body (positive = outward).
    y : float   Longitudinal distance (positive = forward).
    z : float   Depth below body plane (negative = down towards ground).

    Returns
    -------
    (servo_coxa, servo_femur, servo_tibia) : tuple of float
        Angles in servo degrees (90 = neutral).
    \"\"\"
    # --- abs(z) coordinate sign protection ---
    # z must be non-positive (below body plane).  If the caller passes
    # a positive value, force it negative to prevent the IK solver from
    # targeting an impossible point above the chassis.
    z = -abs(z) if z > 0 else z

    # Coxa angle (yaw in horizontal plane)
    coxa_angle_rad = math.atan2(y, x)

    # Distance from the coxa joint to the foot in the horizontal plane
    r = math.sqrt(x**2 + y**2)

    # Horizontal distance from the femur joint to the foot
    l = r - COXA_LENGTH

    # Euclidean distance from femur joint to foot
    d = math.sqrt(l**2 + z**2)

    # Reachability clamp
    max_reach = FEMUR_LENGTH + TIBIA_LENGTH
    if d > max_reach:
        d = max_reach - 0.01

    # Law of cosines for tibia (knee) angle
    cos_tibia = ((FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - d**2) /
                 (2 * FEMUR_LENGTH * TIBIA_LENGTH))
    cos_tibia = max(-1.0, min(1.0, cos_tibia))
    tibia_inner_rad = math.acos(cos_tibia)

    # Law of cosines for femur angle
    cos_femur = ((FEMUR_LENGTH**2 + d**2 - TIBIA_LENGTH**2) /
                 (2 * FEMUR_LENGTH * d))
    cos_femur = max(-1.0, min(1.0, cos_femur))
    femur_inner_rad = math.acos(cos_femur)

    # Angle of the direct line from femur joint to foot (using abs(z)
    # for the vertical component to produce a positive elevation angle,
    # then applying the correct sign via the femur sum below).
    beta_rad = math.atan2(abs(z), l)

    # Final femur angle: elevation to foot + IK interior angle
    femur_angle_rad = beta_rad + femur_inner_rad

    # Convert to degrees
    coxa_angle  = math.degrees(coxa_angle_rad)
    femur_angle = math.degrees(femur_angle_rad)
    tibia_angle = math.degrees(tibia_inner_rad)

    # Map angles to servo ranges (90 is neutral)
    servo_coxa  = 90 + coxa_angle
    servo_femur = 90 + femur_angle
    servo_tibia = 90 + (180 - tibia_angle)

    return servo_coxa, servo_femur, servo_tibia


def inverse_kinematics_2dof(y, z):
    \"\"\"
    Computes joint angles for a 2-DOF leg (no coxa).

    Parameters
    ----------
    y : float   Longitudinal distance (positive = forward).
    z : float   Depth below body plane (negative = down towards ground).

    Returns
    -------
    (servo_femur, servo_tibia) : tuple of float
        Angles in servo degrees (90 = neutral).
    \"\"\"
    # --- abs(z) coordinate sign protection ---
    z = -abs(z) if z > 0 else z

    # Euclidean distance from femur joint to foot in Y-Z plane
    d = math.sqrt(y**2 + z**2)

    max_reach = FEMUR_LENGTH + TIBIA_LENGTH
    if d > max_reach:
        d = max_reach - 0.01

    cos_tibia = ((FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - d**2) /
                 (2 * FEMUR_LENGTH * TIBIA_LENGTH))
    cos_tibia = max(-1.0, min(1.0, cos_tibia))
    tibia_inner_rad = math.acos(cos_tibia)

    cos_femur = ((FEMUR_LENGTH**2 + d**2 - TIBIA_LENGTH**2) /
                 (2 * FEMUR_LENGTH * d))
    cos_femur = max(-1.0, min(1.0, cos_femur))
    femur_inner_rad = math.acos(cos_femur)

    # Use abs(z) for elevation angle to keep beta positive
    beta_rad = math.atan2(abs(z), y)

    femur_angle_rad = beta_rad + femur_inner_rad

    femur_angle = math.degrees(femur_angle_rad)
    tibia_angle = math.degrees(tibia_inner_rad)

    servo_femur = 90 + femur_angle
    servo_tibia = 90 + (180 - tibia_angle)

    return servo_femur, servo_tibia
"""


# ############################################################################
#
#   FILE 7 / 9 :  gait.py
#
# ############################################################################

GAIT_PY = """\
# gait.py
# ============================================================================
# Hexapod Locomotion Engine -- Tripod Gait & Quadruped Strafe
# ============================================================================
#
# Consumes:
#   config.py      -- leg channel map, pulse calibration, geometry constants
#   kinematics.py  -- inverse_kinematics_3dof(x,y,z), inverse_kinematics_2dof(y,z)
#   hardware.py    -- HexapodHardware.set_angle(channel, degrees)
#   remote.py      -- PS5Controller.get_state() -> dict
#
# Called by main.py.  This module owns no threads; it exposes a blocking
# tick-based loop that main.py invokes each cycle.
# ============================================================================

import math
import time
from kinematics import inverse_kinematics_3dof, inverse_kinematics_2dof
import config


# ============================================================================
# Tripod Group Definitions
# ============================================================================
# Standard alternating tripod: Groups swap swing / stance every half-cycle.
#   Group A  (Legs 1-3-5):  front_left,  middle_right,  rear_left
#   Group B  (Legs 2-4-6):  front_right, middle_left,   rear_right

GROUP_A = ['front_left', 'middle_right', 'rear_left']
GROUP_B = ['front_right', 'middle_left', 'rear_right']

# The four 3-DOF corner legs used exclusively during quadruped strafe
QUAD_CORNERS = ['front_left', 'front_right', 'rear_left', 'rear_right']

# The two 2-DOF middle legs that retract during strafe
MIDDLE_LEGS = ['middle_left', 'middle_right']


# ============================================================================
# Angle -> PCA9685 Tick Conversion
# ============================================================================

def angle_to_tick(degrees):
    \"\"\"
    Linearly maps a servo angle (0-180 deg) to a raw PCA9685 12-bit tick
    count using the calibration window defined in config.py.

        0 deg   -> SERVO_MIN_TICK  (102)
        180 deg -> SERVO_MAX_TICK  (492)
    \"\"\"
    clamped = max(0.0, min(180.0, degrees))
    tick = config.SERVO_MIN_TICK + (
        (clamped / 180.0) * (config.SERVO_MAX_TICK - config.SERVO_MIN_TICK)
    )
    return int(round(tick))


# ============================================================================
# Low-Level Leg Command Helpers
# ============================================================================

def command_3dof_leg(hardware, leg_name, x, y, z):
    \"\"\"
    Runs the 3-DOF IK solver for (x, y, z) and writes the resulting angles
    to the three servo channels of the named leg.
    \"\"\"
    channels = config.LEGS[leg_name]['channels']
    coxa_deg, femur_deg, tibia_deg = inverse_kinematics_3dof(x, y, z)

    hardware.set_angle(channels['coxa'],  coxa_deg)
    hardware.set_angle(channels['femur'], femur_deg)
    hardware.set_angle(channels['tibia'], tibia_deg)


def command_2dof_leg(hardware, leg_name, y, z):
    \"\"\"
    Runs the 2-DOF IK solver for (y, z) and writes the resulting angles
    to the two servo channels of the named leg.
    \"\"\"
    channels = config.LEGS[leg_name]['channels']
    femur_deg, tibia_deg = inverse_kinematics_2dof(y, z)

    hardware.set_angle(channels['femur'], femur_deg)
    hardware.set_angle(channels['tibia'], tibia_deg)


def command_leg(hardware, leg_name, x, y, z):
    \"\"\"
    Unified dispatcher -- calls the correct IK solver based on leg type.
    For 2-DOF legs the x component is ignored (they have no coxa).
    \"\"\"
    leg_type = config.LEGS[leg_name]['type']
    if leg_type == '3-dof':
        command_3dof_leg(hardware, leg_name, x, y, z)
    else:
        command_2dof_leg(hardware, leg_name, y, z)


def retract_middle_legs(hardware):
    \"\"\"
    Locks both 2-DOF middle legs in a high retracted position so they
    clear the ground during quadruped strafe manoeuvres.
    \"\"\"
    retract_y = 0.0
    retract_z = -abs(30.0)   # abs(z) sign protection -- always negative
    for leg_name in MIDDLE_LEGS:
        command_2dof_leg(hardware, leg_name, retract_y, retract_z)


def neutralise_all(hardware):
    \"\"\"Commands every leg to the neutral standing pose.\"\"\"
    hardware.set_all_neutral()


# ============================================================================
# Parametric Foot Trajectory Generator
# ============================================================================

def swing_trajectory(t, x_start, y_start, x_end, y_end, z_ground, z_lift):
    \"\"\"
    Parametric swing-phase foot trajectory.

    t       : normalised time 0.0 -> 1.0 across the swing phase
    *_start : foot position at lift-off
    *_end   : foot position at touch-down
    z_ground: planted ground height (negative, e.g. -75)
    z_lift  : apex height during swing (e.g. -35)

    Returns (x, y, z) at time t.

    X and Y interpolate linearly.
    Z follows a half-sine arc:  ground -> apex -> ground.
    \"\"\"
    x = x_start + (x_end - x_start) * t
    y = y_start + (y_end - y_start) * t
    z = z_ground + (z_lift - z_ground) * math.sin(math.pi * t)
    return x, y, z


def stance_trajectory(t, x_start, y_start, x_end, y_end, z_ground):
    \"\"\"
    Parametric stance-phase foot trajectory.

    The foot stays planted on the ground (z = z_ground) and slides
    linearly from start to end as the body moves over it.
    \"\"\"
    x = x_start + (x_end - x_start) * t
    y = y_start + (y_end - y_start) * t
    return x, y, z_ground


# ============================================================================
# TripodGait -- Main Locomotion Controller
# ============================================================================

class TripodGait:
    \"\"\"
    Production-ready tripod gait engine with integrated quadruped strafe mode.

    Call tick() once per main-loop iteration.  It reads the controller
    state, selects the appropriate locomotion mode, computes parametric
    trajectories, and writes servo commands via the hardware driver.

    Timing is self-regulated to stay within the 0.08s - 0.30s window
    per phase step, matching Pi 3B processing limits.
    \"\"\"

    # ------------------------------------------------------------------
    # Geometry defaults (mm) -- override via constructor if needed
    # ------------------------------------------------------------------
    DEFAULT_STANCE_X    =  70.0     # Lateral reach of 3-DOF foot from hip
    DEFAULT_GROUND_Z    = -75.0     # Foot height when planted
    DEFAULT_LIFT_Z      = -35.0     # Foot height at swing apex
    DEFAULT_RETRACT_Z   = -30.0     # Middle-leg retract height (quadruped)
    MAX_STRIDE_Y        =  25.0     # Maximum forward/backward half-stride
    MAX_STRAFE_X        =  25.0     # Maximum lateral half-stride
    MAX_YAW_Y           =  20.0     # Maximum yaw differential half-stride

    # Timing bounds (seconds per parametric sub-step)
    MIN_DELAY = 0.08
    MAX_DELAY = 0.30

    # Number of parametric sub-steps per half-cycle (swing or stance)
    SUBSTEPS = 5

    def __init__(self, hardware):
        self.hw = hardware

        # Tuneable stance parameters with abs(z) sign protection
        self.stance_x  = self.DEFAULT_STANCE_X
        self.ground_z  = -abs(self.DEFAULT_GROUND_Z)    # enforce negative
        self.lift_z    = -abs(self.DEFAULT_LIFT_Z)       # enforce negative
        self.retract_z = -abs(self.DEFAULT_RETRACT_Z)    # enforce negative

        # Phase state: 0 = Group A swings, 1 = Group B swings
        self._half_cycle = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def tick(self, controller_state):
        \"\"\"
        Execute one full half-cycle of the gait.

        Parameters
        ----------
        controller_state : dict
            The snapshot returned by PS5Controller.get_state().

        Returns
        -------
        bool
            False if an emergency stop was triggered (caller should halt).
            True  if the tick completed normally.
        \"\"\"
        # ---- Emergency stop override ----
        if controller_state.get('emergency_stop', False):
            neutralise_all(self.hw)
            return False

        # ---- Read control vectors ----
        fwd   = controller_state.get('left_stick_y',     0.0)
        strf  = controller_state.get('left_stick_x',     0.0)
        yaw   = controller_state.get('right_stick_x',    0.0)
        quad  = controller_state.get('quadruped_strafe',  False)

        magnitude = max(abs(fwd), abs(strf), abs(yaw))

        # ---- Idle -- no meaningful input ----
        if magnitude < 0.05:
            neutralise_all(self.hw)
            self._half_cycle = 0
            return True

        # ---- Select locomotion mode ----
        if quad:
            self._execute_quadruped_strafe(strf)
        else:
            self._execute_tripod_cycle(fwd, strf, yaw, magnitude)

        # Advance half-cycle
        self._half_cycle = (self._half_cycle + 1) % 2
        return True

    # ------------------------------------------------------------------
    # Tripod Gait (6-leg alternating)
    # ------------------------------------------------------------------

    def _execute_tripod_cycle(self, fwd, strf, yaw, magnitude):
        \"\"\"
        One half-cycle of the standard alternating tripod gait.

        Half-cycle 0: Group A swings forward while Group B pushes stance.
        Half-cycle 1: Group B swings forward while Group A pushes stance.
        \"\"\"
        if self._half_cycle == 0:
            swing_group  = GROUP_A
            stance_group = GROUP_B
        else:
            swing_group  = GROUP_B
            stance_group = GROUP_A

        # Compute stride vectors
        stride_y = self.MAX_STRIDE_Y * fwd
        strafe_x = self.MAX_STRAFE_X * strf
        yaw_y    = self.MAX_YAW_Y * yaw

        # Dynamic delay: higher magnitude -> faster cycle (lower delay)
        delay = self.MAX_DELAY / magnitude
        delay = max(self.MIN_DELAY, min(self.MAX_DELAY, delay))
        substep_delay = delay / self.SUBSTEPS

        # ---- Parametric sub-step loop ----
        for i in range(self.SUBSTEPS):
            t = (i + 1) / self.SUBSTEPS     # 0.2, 0.4, 0.6, 0.8, 1.0

            # --- Swing group ---
            for leg_name in swing_group:
                is_middle = 'middle' in leg_name
                is_left   = 'left' in leg_name

                # If strafing is active and this is a 2-DOF middle leg,
                # lift it clear -- it cannot contribute to lateral motion.
                if is_middle and abs(strf) > 0.1:
                    command_2dof_leg(self.hw, leg_name, 0.0, self.retract_z)
                    continue

                # Determine per-leg Y offset (incorporates yaw differential)
                yaw_sign    = 1.0 if is_left else -1.0
                y_end       = stride_y + (yaw_y * yaw_sign)
                y_end       = max(-40.0, min(40.0, y_end))

                # X target (lateral reach + strafe component for 3-DOF)
                if is_middle:
                    x_end = self.stance_x     # 2-DOF: x is ignored by solver
                else:
                    x_end = self.stance_x + strafe_x

                x_end = max(30.0, min(120.0, x_end))

                x, y, z = swing_trajectory(
                    t,
                    x_start  = self.stance_x,
                    y_start  = 0.0,
                    x_end    = x_end,
                    y_end    = y_end,
                    z_ground = self.ground_z,
                    z_lift   = self.lift_z,
                )
                command_leg(self.hw, leg_name, x, y, z)

            # --- Stance group (pushes body in opposite direction) ---
            for leg_name in stance_group:
                is_middle = 'middle' in leg_name
                is_left   = 'left' in leg_name

                if is_middle and abs(strf) > 0.1:
                    command_2dof_leg(self.hw, leg_name, 0.0, self.retract_z)
                    continue

                yaw_sign  = 1.0 if is_left else -1.0
                y_end     = -(stride_y + (yaw_y * yaw_sign))
                y_end     = max(-40.0, min(40.0, y_end))

                if is_middle:
                    x_end = self.stance_x
                else:
                    x_end = self.stance_x - strafe_x

                x_end = max(30.0, min(120.0, x_end))

                x, y, z = stance_trajectory(
                    t,
                    x_start  = self.stance_x,
                    y_start  = 0.0,
                    x_end    = x_end,
                    y_end    = y_end,
                    z_ground = self.ground_z,
                )
                command_leg(self.hw, leg_name, x, y, z)

            time.sleep(substep_delay)

    # ------------------------------------------------------------------
    # Quadruped Strafe Mode (4-leg lateral shuffle)
    # ------------------------------------------------------------------

    def _execute_quadruped_strafe(self, strf):
        \"\"\"
        Side-shuffle using only the four 3-DOF corner legs.

        The two 2-DOF middle legs are locked in a retracted UP position
        for the entire duration of the strafe to prevent mechanical drag.

        Uses a simple 2-phase alternating pattern on the corners:
            Phase 0: front_left + rear_right swing laterally
                     front_right + rear_left push stance
            Phase 1: swap
        \"\"\"
        # Immediately retract middle legs (quadruped isolation)
        retract_middle_legs(self.hw)

        strafe_x = self.MAX_STRAFE_X * strf

        # Split corners into two diagonal pairs
        if self._half_cycle == 0:
            swing_corners  = ['front_left', 'rear_right']
            stance_corners = ['front_right', 'rear_left']
        else:
            swing_corners  = ['front_right', 'rear_left']
            stance_corners = ['front_left', 'rear_right']

        delay = self.MAX_DELAY / max(abs(strf), 0.1)
        delay = max(self.MIN_DELAY, min(self.MAX_DELAY, delay))
        substep_delay = delay / self.SUBSTEPS

        for i in range(self.SUBSTEPS):
            t = (i + 1) / self.SUBSTEPS

            # Keep middle legs retracted every sub-step
            retract_middle_legs(self.hw)

            # Swing pair: lift + move laterally
            for leg_name in swing_corners:
                x_end = self.stance_x + strafe_x
                x_end = max(30.0, min(120.0, x_end))

                x, y, z = swing_trajectory(
                    t,
                    x_start  = self.stance_x,
                    y_start  = 0.0,
                    x_end    = x_end,
                    y_end    = 0.0,
                    z_ground = self.ground_z,
                    z_lift   = self.lift_z,
                )
                command_3dof_leg(self.hw, leg_name, x, y, z)

            # Stance pair: planted, push body in opposite lateral direction
            for leg_name in stance_corners:
                x_end = self.stance_x - strafe_x
                x_end = max(30.0, min(120.0, x_end))

                x, y, z = stance_trajectory(
                    t,
                    x_start  = self.stance_x,
                    y_start  = 0.0,
                    x_end    = x_end,
                    y_end    = 0.0,
                    z_ground = self.ground_z,
                )
                command_3dof_leg(self.hw, leg_name, x, y, z)

            time.sleep(substep_delay)
"""


# ############################################################################
#
#   FILE 8 / 9 :  remote.py
#
# ############################################################################

REMOTE_PY = """\
# remote.py
# ============================================================================
# PS5 DualSense Controller Interface -- evdev / Linux Input Subsystem
# Hardware: Raspberry Pi 3B + PS5 DualSense paired via Bluetooth
# ============================================================================

import evdev
import threading
import time


class PS5Controller:
    \"\"\"
    Reads live Linux event codes from a PS5 DualSense controller using evdev.
    Runs its input polling loop on a daemon thread so it never blocks main.py.

    Stick axes report raw integer values 0-255 with 128 as center.
    A mechanical deadzone of +/-10 raw ticks around center is enforced to
    suppress ghost drift before any value is normalised to the -1.0 ... +1.0
    range consumed by the gait engine.

    Thread Safety
    -------------
    A threading.Lock serialises all writes to stick/button attributes on the
    polling thread and all reads in get_state() on the main thread, ensuring
    the returned dictionary snapshot is always self-consistent even across
    multiple attribute reads.

    Rising-Edge Latch
    -----------------
    The R1 toggle uses explicit previous-state tracking (_r1_prev) to fire
    exactly once per physical button press.  Holding the button down does
    not cause rapid oscillation.
    \"\"\"

    # DualSense raw axis center and range
    _CENTER = 128
    _RANGE  = 128

    # Mechanical deadzone in raw ticks (+/-10 around center)
    _RAW_DEADZONE = 10

    def __init__(self, device_path=None, device_name_hint="Wireless Controller"):
        self._device_path = device_path
        self._device_name_hint = device_name_hint
        self._device = None
        self._running = False

        # Threading lock for cross-thread state consistency
        self._lock = threading.Lock()

        # --- Normalised stick outputs (-1.0 ... +1.0) ---
        self.left_stick_x  = 0.0   # Strafe vector
        self.left_stick_y  = 0.0   # Forward / backward vector (positive = forward)
        self.right_stick_x = 0.0   # Yaw rotation vector

        # --- Button / mode flags ---
        self.ai_assisted_mode = False   # Toggled by R1 (rising edge only)
        self.emergency_stop   = False   # Latched by Circle

        # --- Derived locomotion state ---
        self.quadruped_strafe_mode = False  # True when stick is purely lateral

        # Internal edge-detection state for R1 toggle
        self._r1_prev = False

        # Attempt initial connection
        self._find_device()

    # ------------------------------------------------------------------
    # Device discovery
    # ------------------------------------------------------------------
    def _find_device(self):
        \"\"\"Locate the DualSense on the Linux input subsystem.\"\"\"
        if self._device_path:
            try:
                self._device = evdev.InputDevice(self._device_path)
                print(f"[PS5] Connected via explicit path: "
                      f"{self._device.name} @ {self._device.path}")
                return
            except (FileNotFoundError, PermissionError) as exc:
                print(f"[PS5] Could not open {self._device_path}: {exc}")

        for path in evdev.list_devices():
            dev = evdev.InputDevice(path)
            if self._device_name_hint.lower() in dev.name.lower():
                self._device = dev
                print(f"[PS5] Connected: {dev.name} @ {dev.path}")
                return

        print("[PS5] Controller not found. Will keep scanning in background...")

    # ------------------------------------------------------------------
    # Raw -> normalised conversion with mechanical deadzone
    # ------------------------------------------------------------------
    def _normalise(self, raw_value, invert=False):
        \"\"\"
        Convert a 0-255 raw axis value to -1.0 ... +1.0, collapsing the
        centre +/-_RAW_DEADZONE window to exactly 0.0.
        \"\"\"
        offset = raw_value - self._CENTER

        if abs(offset) <= self._RAW_DEADZONE:
            return 0.0

        normalised = offset / self._RANGE
        normalised = max(-1.0, min(1.0, normalised))

        return -normalised if invert else normalised

    # ------------------------------------------------------------------
    # Quadruped-mode heuristic
    # ------------------------------------------------------------------
    def _update_quadruped_flag(self):
        \"\"\"
        Quadruped side-shuffle mode is active when the left stick is pushed
        predominantly along the X-axis (lateral) with negligible Y-axis
        (forward / backward) input.

        Thresholds:
            |X| > 0.30  AND  |Y| < 0.20  ->  quadruped strafe
        \"\"\"
        self.quadruped_strafe_mode = (
            abs(self.left_stick_x) > 0.30 and abs(self.left_stick_y) < 0.20
        )

    # ------------------------------------------------------------------
    # Background polling thread
    # ------------------------------------------------------------------
    def start(self):
        \"\"\"Spawn a daemon thread that continuously reads controller events.\"\"\"
        self._running = True
        thread = threading.Thread(target=self._poll_loop, daemon=True)
        thread.start()

    def stop(self):
        \"\"\"Signal the polling thread to exit.\"\"\"
        self._running = False

    def _poll_loop(self):
        \"\"\"Main event-reading loop -- runs on the background thread.\"\"\"
        while self._running:
            if self._device is None:
                time.sleep(2)
                self._find_device()
                continue

            try:
                for event in self._device.read_loop():
                    if not self._running:
                        break

                    # ---- Analog Axes ----
                    if event.type == evdev.ecodes.EV_ABS:

                        # Left Stick X  ->  strafe
                        if event.code == evdev.ecodes.ABS_X:
                            with self._lock:
                                self.left_stick_x = self._normalise(event.value)
                                self._update_quadruped_flag()

                        # Left Stick Y  ->  forward / backward (inverted)
                        elif event.code == evdev.ecodes.ABS_Y:
                            with self._lock:
                                self.left_stick_y = self._normalise(
                                    event.value, invert=True
                                )
                                self._update_quadruped_flag()

                        # Right Stick X  ->  yaw rotation
                        elif event.code in (evdev.ecodes.ABS_Z,
                                            evdev.ecodes.ABS_RX):
                            with self._lock:
                                self.right_stick_x = self._normalise(event.value)

                    # ---- Digital Buttons ----
                    elif event.type == evdev.ecodes.EV_KEY:

                        # Circle / BTN_EAST  ->  Emergency Stop (instant latch)
                        if event.code in (evdev.ecodes.BTN_EAST, 305, 304):
                            if event.value == 1:
                                with self._lock:
                                    self.emergency_stop = True

                        # R1 / BTN_TR  ->  AI Assisted Mode toggle
                        # TRUE RISING-EDGE LATCH: fires exactly once per
                        # physical button press.  Holding the button down
                        # does NOT cause repeated toggles.
                        elif event.code in (evdev.ecodes.BTN_TR, 311):
                            r1_now = bool(event.value)
                            with self._lock:
                                if r1_now and not self._r1_prev:
                                    self.ai_assisted_mode = not self.ai_assisted_mode
                                    state_str = ("ENABLED" if self.ai_assisted_mode
                                                 else "DISABLED")
                                    print(f"[PS5] AI Assisted Mode: {state_str}")
                                self._r1_prev = r1_now

            except (OSError, IOError):
                print("[PS5] Controller disconnected -- zeroing inputs.")
                with self._lock:
                    self._device = None
                    self.left_stick_x  = 0.0
                    self.left_stick_y  = 0.0
                    self.right_stick_x = 0.0
                    self.quadruped_strafe_mode = False

    # ------------------------------------------------------------------
    # Public state snapshot -- consumed by main.py / gait.py
    # ------------------------------------------------------------------
    def get_state(self):
        \"\"\"
        Returns a dictionary snapshot of the current control vectors and
        mode flags.  Thread-safe: acquires the internal Lock to guarantee
        the snapshot is self-consistent across all fields.
        \"\"\"
        with self._lock:
            return {
                'left_stick_x':        self.left_stick_x,
                'left_stick_y':        self.left_stick_y,
                'right_stick_x':       self.right_stick_x,
                'quadruped_strafe':    self.quadruped_strafe_mode,
                'ai_assist_enabled':   self.ai_assisted_mode,
                'emergency_stop':      self.emergency_stop,
            }
"""


# ############################################################################
#
#   FILE 9 / 9 :  main.py
#
# ############################################################################

MAIN_PY = """\
# main.py
# ============================================================================
# Hexapod Robot — Final Orchestration Engine (Production Hardened)
# Hardware: Raspberry Pi 3B + PCA9685 + PS5 DualSense + HC-SR04 + MPU6050
# ============================================================================

import time
import sys
import math
import threading
import numpy as np
from sklearn.linear_model import SGDClassifier

import config
from hardware import HexapodHardware
from gait import TripodGait, neutralise_all, command_3dof_leg
from remote import PS5Controller
from video_stream import start_stream_thread
from sensors import UltrasonicSensor, Gyroscope


# ============================================================================
# Sensor Fusion — Coherent Multi-Sensor Pooling
# ============================================================================

class SensorFusion:
    \"\"\"
    Reads all onboard sensors in a single sequential call to produce a
    temporally coherent feature snapshot for the ML safety classifier.
    \"\"\"

    COLLISION_THRESHOLD_CM = 15.0
    CATASTROPHIC_TILT_DEG  = 30.0

    def __init__(self):
        print("[Sensors] Initializing HC-SR04 ultrasonic (GPIO 17/27)...")
        self.ultrasonic = UltrasonicSensor(trigger_pin=17, echo_pin=27)

        print("[Sensors] Initializing MPU6050 gyroscope (I2C 0x68)...")
        self.gyro = Gyroscope(address=0x68)

        self._prev_distance = 400.0
        self._distance      = 400.0
        self._pitch          = 0.0
        self._roll           = 0.0

    def read_all(self):
        self._prev_distance = self._distance
        self._distance      = self.ultrasonic.get_distance_cm()
        self._pitch, self._roll = self.gyro.get_orientation()

        return {
            'distance_cm':      self._distance,
            'prev_distance_cm': self._prev_distance,
            'pitch_deg':        self._pitch,
            'roll_deg':         self._roll,
        }

    def feature_vector(self):
        return np.array([[self._distance, self._pitch, self._roll]])

    def is_collision_imminent(self):
        return self._distance < self.COLLISION_THRESHOLD_CM

    def is_catastrophic_tilt(self):
        return abs(self._pitch) > self.CATASTROPHIC_TILT_DEG or \
               abs(self._roll) > self.CATASTROPHIC_TILT_DEG

    def detected_dynamic_object(self):
        return (self._prev_distance >= self.COLLISION_THRESHOLD_CM and
                self._distance < self.COLLISION_THRESHOLD_CM)


# ============================================================================
# ML Safety Classifier — Online SGD with Scikit-Learn
# ============================================================================

class SafetyClassifier:
    \"\"\"
    Lightweight online-learning binary classifier (Safe=1 / Unsafe=0)
    using Stochastic Gradient Descent with logistic loss.
    \"\"\"

    SAFE_TRAINING_INTERVAL = 50

    def __init__(self):
        print("[AI] Initializing SGDClassifier safety net...")
        self.model = SGDClassifier(
            loss='log_loss',
            learning_rate='constant',
            eta0=0.01,
            random_state=42,
        )

        print("[AI] Seeding classifier with safe/unsafe baseline vectors...")
        x_seed = np.array([
            [200.0,   0.0,   0.0],
            [100.0,   5.0,   3.0],
            [ 50.0,  10.0,   8.0],
            [ 25.0,   0.0,   0.0],
            [  5.0,  45.0,  45.0],
            [  3.0,   0.0,   0.0],
            [ 10.0,  35.0,  20.0],
            [  8.0,  40.0,  30.0],
        ])
        y_seed = np.array([1, 1, 1, 1, 0, 0, 0, 0])
        self.model.partial_fit(x_seed, y_seed, classes=np.array([0, 1]))

        self._safe_counter = 0
        print("[AI] Safety classifier online and ready.")

    def train_unsafe(self, feature_vector):
        self.model.partial_fit(feature_vector, np.array([0]))

    def train_safe(self, feature_vector):
        self._safe_counter += 1
        if self._safe_counter >= self.SAFE_TRAINING_INTERVAL:
            self._safe_counter = 0
            self.model.partial_fit(feature_vector, np.array([1]))

    def predict_safe(self, feature_vector):
        return bool(self.model.predict(feature_vector)[0] == 1)


# ============================================================================
# Friendly Leg Wave Animation
# ============================================================================

def leg_wave_animation(hardware):
    \"\"\"
    Executes a friendly wave sequence with localized hardware fault protection.
    \"\"\"
    print("[Animation] Friendly leg wave — dynamic object detected!")
    
    try:
        fl_channels = config.LEGS['front_left']['channels']
        wave_coxa_ch  = fl_channels['coxa']
        wave_femur_ch = fl_channels['femur']
        wave_tibia_ch = fl_channels['tibia']

        hardware.set_angle(wave_femur_ch, 45)
        time.sleep(0.3)

        for wave_cycle in range(3):
            hardware.set_angle(wave_coxa_ch, 60)
            hardware.set_angle(wave_tibia_ch, 50)
            time.sleep(0.2)

            hardware.set_angle(wave_coxa_ch, 120)
            hardware.set_angle(wave_tibia_ch, 130)
            time.sleep(0.2)

            hardware.set_angle(wave_coxa_ch, 60)
            hardware.set_angle(wave_tibia_ch, 50)
            time.sleep(0.2)

            hardware.set_angle(wave_coxa_ch, 90)
            hardware.set_angle(wave_tibia_ch, 90)
            time.sleep(0.2)

        hardware.set_angle(wave_femur_ch, 90)
        time.sleep(0.3)
        print("[Animation] Wave complete cleanly.")
        
    except Exception as exc:
        print(f"[ERROR] Leg wave animation hardware write interrupted: {exc}")


# ============================================================================
# Main Orchestration
# ============================================================================

def main():
    print("=" * 68)
    print("  HEXAPOD ADVANCED AUTONOMOUS AGENT")
    print("=" * 68)

    # Phase 1 — Start Flask FPV Video Stream
    print("\\n[Boot] Phase 1: Starting Flask FPV stream server...")
    start_stream_thread()

    # Phase 2 — Initialise PS5 DualSense Controller
    print("\\n[Boot] Phase 2: Connecting PS5 DualSense controller...")
    controller = PS5Controller(device_name_hint="Wireless Controller")
    controller.start()

    # Phase 3 — Initialise PCA9685 Servo Hardware
    print("\\n[Boot] Phase 3: Initializing PCA9685 servo driver...")
    hardware = None
    try:
        hardware = HexapodHardware()
    except Exception as exc:
        print(f"[FATAL] PCA9685 hardware init failed: {exc}")
        controller.stop()
        sys.exit(1)

    # Phase 4 — Initialise Sensor Fusion Engine
    print("\\n[Boot] Phase 4: Initializing sensor fusion engine...")
    sensors = SensorFusion()

    # Phase 5 — Initialise ML Safety Classifier
    print("\\n[Boot] Phase 5: Initializing ML safety classifier...")
    classifier = SafetyClassifier()

    # Phase 6 — Stance Calibration
    print("\\n[Boot] Phase 6: Executing neutral stance calibration...")
    hardware.set_all_neutral()
    time.sleep(2.0)

    # Phase 7 — Instantiate Gait Engine
    print("\\n[Boot] Phase 7: Instantiating tripod gait engine...")
    gait = TripodGait(hardware)

    print("\\n" + "=" * 68)
    print("  ALL SYSTEMS ONLINE — ENTERING MAIN LOCOMOTION LOOP")
    print("=" * 68 + "\\n")

    loop_counter = 0
    telemetry_interval = 100
    
    # Define system loop speed fallbacks if not explicitly found in config
    loop_delay = getattr(config, 'LOOP_DELAY', 0.02) 

    try:
        while True:
            loop_counter += 1

            # ---- Step 1: Read controller state ----
            ctrl = controller.get_state()

            # ---- Step 2: Emergency stop check ----
            if ctrl['emergency_stop']:
                print("\\n[E-Stop] Emergency Stop Triggered! Shutting down...")
                neutralise_all(hardware)
                time.sleep(0.2)
                hardware.deinit()
                controller.stop()
                sys.exit(0)

            # ---- Step 3: Coherent sensor read ----
            sensor_data = sensors.read_all()
            features    = sensors.feature_vector()

            # ---- Step 4: Extract control vectors ----
            forward_vel = ctrl['left_stick_y']
            strafe_vel  = ctrl['left_stick_x']
            yaw_vel     = ctrl['right_stick_x']
            ai_active   = ctrl['ai_assist_enabled']

            # ---- Step 5: Catastrophic tilt detection & Loop Throttle Guard ----
            if sensors.is_catastrophic_tilt():
                print(f"[SAFETY] Catastrophic tilt! Pitch={sensor_data['pitch_deg']:.1f}°")
                classifier.train_unsafe(features)
                neutralise_all(hardware)
                time.sleep(loop_delay)  # Protection boundary against high-speed CPU cycles
                continue

            # ---- Step 6: Targeted ML training guardrail against class drift ----
            if sensors.is_collision_imminent():
                classifier.train_unsafe(features)
            else:
                # Anti-Poisoning Filter: Only train safe if user is moving the sticks
                if abs(forward_vel) > 0.05 or abs(strafe_vel) > 0.05 or abs(yaw_vel) > 0.05:
                    classifier.train_safe(features)

            # ---- Step 7: AI-Assisted mode overrides ----
            if ai_active:
                ml_safe = classifier.predict_safe(features)

                if sensors.detected_dynamic_object():
                    print(f"[AI] Dynamic object intersection at {sensor_data['distance_cm']:.1f} cm!")
                    neutralise_all(hardware)
                    leg_wave_animation(hardware)
                    neutralise_all(hardware)
                    time.sleep(loop_delay)  # Protect Core Compute
                    continue

                if sensors.is_collision_imminent() and forward_vel > 0.1:
                    print(f"[AI] Collision Imminent. Dampening forward velocity.")
                    forward_vel = 0.0

                if not ml_safe:
                    print("[AI] Classifier evaluated environment as UNSAFE. Locomotion halted.")
                    forward_vel = 0.0
                    strafe_vel  = 0.0
                    yaw_vel     = 0.0

            # ---- Step 8: Build modified controller state ----
            gait_state = {
                'left_stick_x':      strafe_vel,
                'left_stick_y':      forward_vel,
                'right_stick_x':     yaw_vel,
                'quadruped_strafe':  ctrl['quadruped_strafe'],
                'ai_assist_enabled': ai_active,
                'emergency_stop':    False,
            }

            # ---- Step 9: Execute one gait tick ----
            tick_ok = gait.tick(gait_state)
            if not tick_ok:
                print("[Gait] Failure inside gait tick engine. Breaking execution loop.")
                break

            # ---- Step 10: Periodic telemetry logging ----
            if loop_counter % telemetry_interval == 0:
                ai_status = "ON" if ai_active else "OFF"
                mode_str  = "QUAD-STRAFE" if ctrl['quadruped_strafe'] else "TRIPOD"
                print(f"[Telemetry #{loop_counter}] Dist={sensor_data['distance_cm']:5.1f}cm | "
                      f"Pitch={sensor_data['pitch_deg']:+5.1f}° | Mode={mode_str} | AI={ai_status}")

    except KeyboardInterrupt:
        print("\\n[Shutdown] KeyboardInterrupt received. Cleaning environment...")

    finally:
        print("[Shutdown] Releasing active hardware layers...")
        if hardware:
            try:
                neutralise_all(hardware)
                time.sleep(0.2)
                hardware.deinit()
            except Exception:
                pass
        controller.stop()
        print("[Shutdown] Core engine offline.")


# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    main()
"""


# ############################################################################
#
#   BUILD SCRIPT EXECUTION
#
# ############################################################################

FILES = [
    ("requirements.txt", REQUIREMENTS_TXT),
    ("config.py",        CONFIG_PY),
    ("hardware.py",      HARDWARE_PY),
    ("sensors.py",       SENSORS_PY),
    ("video_stream.py",  VIDEO_STREAM_PY),
    ("kinematics.py",    KINEMATICS_PY),
    ("gait.py",          GAIT_PY),
    ("remote.py",        REMOTE_PY),
    ("main.py",          MAIN_PY),
]


def main():
    print("=" * 68)
    print("  HEXAPOD WORKSPACE BUILDER")
    print(f"  Target directory: ./{WORKSPACE}/")
    print("=" * 68)

    os.makedirs(WORKSPACE, exist_ok=True)

    for filename, content in FILES:
        filepath = os.path.join(WORKSPACE, filename)
        with open(filepath, "w", encoding="utf-8", newline="\n") as f:
            f.write(content)
        size = len(content.encode("utf-8"))
        print(f"  [OK] {filepath:<35s}  ({size:>6,d} bytes)")

    print()
    print("-" * 68)
    print(f"  {len(FILES)} files written to ./{WORKSPACE}/")
    print()
    print("  Next steps:")
    print(f"    1.  cd {WORKSPACE}")
    print("    2.  pip install -r requirements.txt")
    print("    3.  sudo python main.py")
    print("-" * 68)


if __name__ == "__main__":
    main()
