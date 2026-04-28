import math

class RobotP3DX:
    """
    Base class for the Pioneer P3DX robot. 
    Handles basic interactions with CoppeliaSim like getting positions and setting motor speeds.
    """
    def __init__(self, sim, robot_name):
        self.sim = sim
        self.name = robot_name
        
        # Get handles for the robot base and motors
        self.base_handle = self.sim.getObject(f'/{self.name}')
        self.left_motor = self.sim.getObject(f'/{self.name}/leftMotor')
        self.right_motor = self.sim.getObject(f'/{self.name}/rightMotor')

    def get_position(self):
        """Returns the [x, y, z] position of the robot."""
        return self.sim.getObjectPosition(self.base_handle, self.sim.handle_world)

    def get_yaw(self):
        """Returns the current yaw (rotation around Z-axis) of the robot."""
        orientation = self.sim.getObjectOrientation(self.base_handle, self.sim.handle_world)
        return orientation[2] # Z-axis rotation

    def set_velocities(self, v_left, v_right):
        """Sets the target velocity for the left and right motors."""
        self.sim.setJointTargetVelocity(self.left_motor, v_left)
        self.sim.setJointTargetVelocity(self.right_motor, v_right)

    def step(self):
        """
        Abstract method to be overridden by child classes. 
        Defines the behavior of the robot for a single simulation step.
        """
        pass