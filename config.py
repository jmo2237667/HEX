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
