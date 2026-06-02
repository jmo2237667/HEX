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
    """
    Linearly maps a servo angle (0-180 deg) to a raw PCA9685 12-bit tick
    count using the calibration window defined in config.py.

        0 deg   -> SERVO_MIN_TICK  (102)
        180 deg -> SERVO_MAX_TICK  (492)
    """
    clamped = max(0.0, min(180.0, degrees))
    tick = config.SERVO_MIN_TICK + (
        (clamped / 180.0) * (config.SERVO_MAX_TICK - config.SERVO_MIN_TICK)
    )
    return int(round(tick))


# ============================================================================
# Low-Level Leg Command Helpers
# ============================================================================

def command_3dof_leg(hardware, leg_name, x, y, z):
    """
    Runs the 3-DOF IK solver for (x, y, z) and writes the resulting angles
    to the three servo channels of the named leg.
    """
    channels = config.LEGS[leg_name]['channels']
    coxa_deg, femur_deg, tibia_deg = inverse_kinematics_3dof(x, y, z)

    hardware.set_angle(channels['coxa'],  coxa_deg)
    hardware.set_angle(channels['femur'], femur_deg)
    hardware.set_angle(channels['tibia'], tibia_deg)


def command_2dof_leg(hardware, leg_name, y, z):
    """
    Runs the 2-DOF IK solver for (y, z) and writes the resulting angles
    to the two servo channels of the named leg.
    """
    channels = config.LEGS[leg_name]['channels']
    femur_deg, tibia_deg = inverse_kinematics_2dof(y, z)

    hardware.set_angle(channels['femur'], femur_deg)
    hardware.set_angle(channels['tibia'], tibia_deg)


def command_leg(hardware, leg_name, x, y, z):
    """
    Unified dispatcher -- calls the correct IK solver based on leg type.
    For 2-DOF legs the x component is ignored (they have no coxa).
    """
    leg_type = config.LEGS[leg_name]['type']
    if leg_type == '3-dof':
        command_3dof_leg(hardware, leg_name, x, y, z)
    else:
        command_2dof_leg(hardware, leg_name, y, z)


def retract_middle_legs(hardware):
    """
    Locks both 2-DOF middle legs in a high retracted position so they
    clear the ground during quadruped strafe manoeuvres.
    """
    retract_y = 0.0
    retract_z = -abs(30.0)   # abs(z) sign protection -- always negative
    for leg_name in MIDDLE_LEGS:
        command_2dof_leg(hardware, leg_name, retract_y, retract_z)


def neutralise_all(hardware):
    """Commands every leg to the neutral standing pose."""
    hardware.set_all_neutral()


# ============================================================================
# Parametric Foot Trajectory Generator
# ============================================================================

def swing_trajectory(t, x_start, y_start, x_end, y_end, z_ground, z_lift):
    """
    Parametric swing-phase foot trajectory.

    t       : normalised time 0.0 -> 1.0 across the swing phase
    *_start : foot position at lift-off
    *_end   : foot position at touch-down
    z_ground: planted ground height (negative, e.g. -75)
    z_lift  : apex height during swing (e.g. -35)

    Returns (x, y, z) at time t.

    X and Y interpolate linearly.
    Z follows a half-sine arc:  ground -> apex -> ground.
    """
    x = x_start + (x_end - x_start) * t
    y = y_start + (y_end - y_start) * t
    z = z_ground + (z_lift - z_ground) * math.sin(math.pi * t)
    return x, y, z


def stance_trajectory(t, x_start, y_start, x_end, y_end, z_ground):
    """
    Parametric stance-phase foot trajectory.

    The foot stays planted on the ground (z = z_ground) and slides
    linearly from start to end as the body moves over it.
    """
    x = x_start + (x_end - x_start) * t
    y = y_start + (y_end - y_start) * t
    return x, y, z_ground


# ============================================================================
# TripodGait -- Main Locomotion Controller
# ============================================================================

class TripodGait:
    """
    Production-ready tripod gait engine with integrated quadruped strafe mode.

    Call tick() once per main-loop iteration.  It reads the controller
    state, selects the appropriate locomotion mode, computes parametric
    trajectories, and writes servo commands via the hardware driver.

    Timing is self-regulated to stay within the 0.08s - 0.30s window
    per phase step, matching Pi 3B processing limits.
    """

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
        """
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
        """
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
        """
        One half-cycle of the standard alternating tripod gait.

        Half-cycle 0: Group A swings forward while Group B pushes stance.
        Half-cycle 1: Group B swings forward while Group A pushes stance.
        """
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
        """
        Side-shuffle using only the four 3-DOF corner legs.

        The two 2-DOF middle legs are locked in a retracted UP position
        for the entire duration of the strafe to prevent mechanical drag.

        Uses a simple 2-phase alternating pattern on the corners:
            Phase 0: front_left + rear_right swing laterally
                     front_right + rear_left push stance
            Phase 1: swap
        """
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
