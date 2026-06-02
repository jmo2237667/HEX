# remote.py
# ============================================================================
# PS5 DualSense Controller Interface -- evdev / Linux Input Subsystem
# Hardware: Raspberry Pi 3B + PS5 DualSense paired via Bluetooth
# ============================================================================

import evdev
import threading
import time


class PS5Controller:
    """
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
    """

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
        """Locate the DualSense on the Linux input subsystem."""
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
        """
        Convert a 0-255 raw axis value to -1.0 ... +1.0, collapsing the
        centre +/-_RAW_DEADZONE window to exactly 0.0.
        """
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
        """
        Quadruped side-shuffle mode is active when the left stick is pushed
        predominantly along the X-axis (lateral) with negligible Y-axis
        (forward / backward) input.

        Thresholds:
            |X| > 0.30  AND  |Y| < 0.20  ->  quadruped strafe
        """
        self.quadruped_strafe_mode = (
            abs(self.left_stick_x) > 0.30 and abs(self.left_stick_y) < 0.20
        )

    # ------------------------------------------------------------------
    # Background polling thread
    # ------------------------------------------------------------------
    def start(self):
        """Spawn a daemon thread that continuously reads controller events."""
        self._running = True
        thread = threading.Thread(target=self._poll_loop, daemon=True)
        thread.start()

    def stop(self):
        """Signal the polling thread to exit."""
        self._running = False

    def _poll_loop(self):
        """Main event-reading loop -- runs on the background thread."""
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
        """
        Returns a dictionary snapshot of the current control vectors and
        mode flags.  Thread-safe: acquires the internal Lock to guarantee
        the snapshot is self-consistent across all fields.
        """
        with self._lock:
            return {
                'left_stick_x':        self.left_stick_x,
                'left_stick_y':        self.left_stick_y,
                'right_stick_x':       self.right_stick_x,
                'quadruped_strafe':    self.quadruped_strafe_mode,
                'ai_assist_enabled':   self.ai_assisted_mode,
                'emergency_stop':      self.emergency_stop,
            }
