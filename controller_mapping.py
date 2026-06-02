import pygame

class DualSenseController:
    def __init__(self, deadzone=0.1):
        self.deadzone = deadzone
        self.joystick = None
        
        # State tracking for toggle functionality
        self.ai_assist_enabled = False
        self.r1_previously_pressed = False
        
        self.init_controller()
        
    def init_controller(self):
        """Initializes pygame joystick module and detects the controller."""
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print(f"Controller detected: {self.joystick.get_name()}")
            else:
                print("No controller found. Please connect a PS5 DualSense controller.")
        except Exception as e:
            print(f"Error initializing controller: {e}")

    def apply_deadzone(self, value):
        """Applies a software deadzone to prevent stick drift."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def get_inputs(self):
        """
        Polls pygame events and returns the mapped control dictionary.
        Must be called continuously in the main loop.
        """
        # Process pygame event queue (required to get updated joystick state)
        for event in pygame.event.get():
            pass
            
        inputs = {
            'forward_backward': 0.0,
            'strafe': 0.0,
            'yaw': 0.0,
            'ai_assist_enabled': self.ai_assist_enabled,
            'emergency_stop': False
        }
        
        if not self.joystick:
            return inputs
            
        # Left Stick Vertical (Axis 1) - Forward/Backward
        inputs['forward_backward'] = self.apply_deadzone(self.joystick.get_axis(1))
        
        # Left Stick Horizontal (Axis 0) - Strafing Left/Right
        inputs['strafe'] = self.apply_deadzone(self.joystick.get_axis(0))
        
        # Right Stick Horizontal (Axis 2) - Yaw Rotation
        inputs['yaw'] = self.apply_deadzone(self.joystick.get_axis(2))
        
        # Button 1 (Circle / O) - Emergency Stop
        inputs['emergency_stop'] = self.joystick.get_button(1)
        
        # Button 10 (Right Bumper / R1) - AI Assist Toggle
        r1_currently_pressed = self.joystick.get_button(10)
        
        # Toggle logic (rising edge detection)
        if r1_currently_pressed and not self.r1_previously_pressed:
            self.ai_assist_enabled = not self.ai_assist_enabled
            inputs['ai_assist_enabled'] = self.ai_assist_enabled
            print(f"Assisted Driving AI Agent: {'ENABLED' if self.ai_assist_enabled else 'DISABLED'}")
            
        self.r1_previously_pressed = r1_currently_pressed
        
        return inputs
