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
#
# Output:
#   Angles are returned in Webots-compatible Radians (where 0.0 is neutral),
#   converted directly from the physical 90-degree servo center logic.
# ============================================================================

import math
from config import COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH


def inverse_kinematics_3dof(x, y, z):
    """
    Computes joint angles for a 3-DOF leg.

    Parameters
    ----------
    x : float   Lateral distance outward from the body (positive = outward).
    y : float   Longitudinal distance (positive = forward).
    z : float   Depth below body plane (negative = down towards ground).

    Returns
    -------
    (webots_coxa, webots_femur, webots_tibia) : tuple of float
        Angles in radians mapped for Webots HingeJoints (0.0 = neutral).
    """
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

    # Convert to degrees for physical servo mapping logic
    coxa_angle  = math.degrees(coxa_angle_rad)
    femur_angle = math.degrees(femur_angle_rad)
    tibia_angle = math.degrees(tibia_inner_rad)

    # Map angles to physical servo ranges (90 is neutral)
    servo_coxa  = 90 + coxa_angle
    servo_femur = 90 + femur_angle
    servo_tibia = 90 + (180 - tibia_angle)

    # --- WEBOTS CONVERSION ---
    # Convert physical degrees to Webots radians (Webots neutral = 0.0 rad)
    webots_coxa  = math.radians(servo_coxa - 90.0)
    webots_femur = math.radians(servo_femur - 90.0)
    webots_tibia = math.radians(servo_tibia - 90.0)

    return webots_coxa, webots_femur, webots_tibia


def inverse_kinematics_2dof(y, z):
    """
    Computes joint angles for a 2-DOF leg (no coxa).

    Parameters
    ----------
    y : float   Longitudinal distance (positive = forward).
    z : float   Depth below body plane (negative = down towards ground).

    Returns
    -------
    (webots_femur, webots_tibia) : tuple of float
        Angles in radians mapped for Webots HingeJoints (0.0 = neutral).
    """
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

    # Convert to degrees for physical servo mapping logic
    femur_angle = math.degrees(femur_angle_rad)
    tibia_angle = math.degrees(tibia_inner_rad)

    # Map angles to physical servo ranges (90 is neutral)
    servo_femur = 90 + femur_angle
    servo_tibia = 90 + (180 - tibia_angle)

    # --- WEBOTS CONVERSION ---
    # Convert physical degrees to Webots radians (Webots neutral = 0.0 rad)
    webots_femur = math.radians(servo_femur - 90.0)
    webots_tibia = math.radians(servo_tibia - 90.0)

    return webots_femur, webots_tibia