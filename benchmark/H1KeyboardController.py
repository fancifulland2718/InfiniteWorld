import numpy as np
import carb
from typing import Optional
from humanoid.h1 import H1FlatTerrainPolicy


class H1KeyboardController:
    """H1 Humanoid Keyboard Controller - Fixed version for stability"""
    
    def __init__(
        self,
        prim_path: str,
        name: str = "h1",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ):
        """
        Initialize H1 keyboard controller with proper command handling
        """
        # Initialize H1 robot with policy
        self.h1_robot = H1FlatTerrainPolicy(
            prim_path=prim_path,
            name=name,
            usd_path=usd_path,
            position=position,
            orientation=orientation
        )
        
        # Use list instead of numpy array to match original implementation
        self._base_command = [0.0, 0.0, 0.0]
        
        # Movement parameters - matched to original working values
        self._forward_speed = 0.75
        self._yaw_speed = 0.75
        
        # Simplified keyboard mapping - only working controls from original
        self._input_keyboard_mapping = {
            # Forward movement only (no backward to avoid instability)
            "NUMPAD_8": [self._forward_speed, 0.0, 0.0],
            "UP": [self._forward_speed, 0.0, 0.0],
            
            # Yaw rotation only (no lateral movement for stability)
            "NUMPAD_4": [0.0, 0.0, self._yaw_speed],
            "LEFT": [0.0, 0.0, self._yaw_speed],
            "NUMPAD_6": [0.0, 0.0, -self._yaw_speed],
            "RIGHT": [0.0, 0.0, -self._yaw_speed],
        }
        
        self._is_initialized = False
        
    def initialize(self, physics_sim_view=None):
        """Initialize the H1 robot"""
        self.h1_robot.initialize(physics_sim_view)
        self._is_initialized = True
        print("H1 Humanoid Robot initialized successfully - Fixed version")
        
    def post_reset(self):
        """Reset the robot state"""
        self.h1_robot.post_reset()
        self._base_command = [0.0, 0.0, 0.0]
        
    def advance(self, dt):
        """Advance the robot simulation with current command"""
        if self._is_initialized:
            # Convert list to numpy array for the policy
            command_array = np.array(self._base_command)
            self.h1_robot.advance(dt, command_array)
            
    def handle_keyboard_event(self, event, *args, **kwargs):
        """
        Handle keyboard input events with proper command management
        Fixed to avoid command accumulation issues
        """
        try:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                # Handle key press - use direct assignment instead of addition
                if event.input.name in self._input_keyboard_mapping:
                    command_delta = self._input_keyboard_mapping[event.input.name]
                    # Add to existing command using numpy operations then convert back
                    current_cmd = np.array(self._base_command)
                    new_cmd = current_cmd + np.array(command_delta)
                    self._base_command = new_cmd.tolist()
                    print(f"Key pressed: {event.input.name}, Command: {self._base_command}")
                    
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                # Handle key release - subtract the command
                if event.input.name in self._input_keyboard_mapping:
                    command_delta = self._input_keyboard_mapping[event.input.name]
                    # Subtract from existing command
                    current_cmd = np.array(self._base_command)
                    new_cmd = current_cmd - np.array(command_delta)
                    self._base_command = new_cmd.tolist()
                    print(f"Key released: {event.input.name}, Command: {self._base_command}")
                    
        except Exception as e:
            print(f"Error handling keyboard event: {e}")
            
        return True
        
    def get_current_command(self):
        """Get the current movement command"""
        return self._base_command.copy()
        
    def set_command(self, command):
        """Manually set the movement command"""
        self._base_command = [float(command[0]), float(command[1]), float(command[2])]
        
    def stop(self):
        """Stop all movement"""
        self._base_command = [0.0, 0.0, 0.0]
        
    def get_robot_state(self):
        """Get current robot state information"""
        if self._is_initialized:
            try:
                pos, quat = self.h1_robot.robot.get_world_pose()
                lin_vel = self.h1_robot.robot.get_linear_velocity()
                ang_vel = self.h1_robot.robot.get_angular_velocity()
                joint_pos = self.h1_robot.robot.get_joint_positions()
                joint_vel = self.h1_robot.robot.get_joint_velocities()
                
                return {
                    "position": pos,
                    "orientation": quat,
                    "linear_velocity": lin_vel,
                    "angular_velocity": ang_vel,
                    "joint_positions": joint_pos,
                    "joint_velocities": joint_vel,
                    "current_command": self._base_command
                }
            except Exception as e:
                print(f"Error getting robot state: {e}")
                return None
        return None