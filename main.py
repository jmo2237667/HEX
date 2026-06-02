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
    """
    Reads all onboard sensors in a single sequential call to produce a
    temporally coherent feature snapshot for the ML safety classifier.
    """

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
        return abs(self._pitch) > self.CATASTROPHIC_TILT_DEG or                abs(self._roll) > self.CATASTROPHIC_TILT_DEG

    def detected_dynamic_object(self):
        return (self._prev_distance >= self.COLLISION_THRESHOLD_CM and
                self._distance < self.COLLISION_THRESHOLD_CM)


# ============================================================================
# ML Safety Classifier — Online SGD with Scikit-Learn
# ============================================================================

class SafetyClassifier:
    """
    Lightweight online-learning binary classifier (Safe=1 / Unsafe=0)
    using Stochastic Gradient Descent with logistic loss.
    """

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
    """
    Executes a friendly wave sequence with localized hardware fault protection.
    """
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
    print("\n[Boot] Phase 1: Starting Flask FPV stream server...")
    start_stream_thread()

    # Phase 2 — Initialise PS5 DualSense Controller
    print("\n[Boot] Phase 2: Connecting PS5 DualSense controller...")
    controller = PS5Controller(device_name_hint="Wireless Controller")
    controller.start()

    # Phase 3 — Initialise PCA9685 Servo Hardware
    print("\n[Boot] Phase 3: Initializing PCA9685 servo driver...")
    hardware = None
    try:
        hardware = HexapodHardware()
    except Exception as exc:
        print(f"[FATAL] PCA9685 hardware init failed: {exc}")
        controller.stop()
        sys.exit(1)

    # Phase 4 — Initialise Sensor Fusion Engine
    print("\n[Boot] Phase 4: Initializing sensor fusion engine...")
    sensors = SensorFusion()

    # Phase 5 — Initialise ML Safety Classifier
    print("\n[Boot] Phase 5: Initializing ML safety classifier...")
    classifier = SafetyClassifier()

    # Phase 6 — Stance Calibration
    print("\n[Boot] Phase 6: Executing neutral stance calibration...")
    hardware.set_all_neutral()
    time.sleep(2.0)

    # Phase 7 — Instantiate Gait Engine
    print("\n[Boot] Phase 7: Instantiating tripod gait engine...")
    gait = TripodGait(hardware)

    print("\n" + "=" * 68)
    print("  ALL SYSTEMS ONLINE — ENTERING MAIN LOCOMOTION LOOP")
    print("=" * 68 + "\n")

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
                print("\n[E-Stop] Emergency Stop Triggered! Shutting down...")
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
        print("\n[Shutdown] KeyboardInterrupt received. Cleaning environment...")

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
