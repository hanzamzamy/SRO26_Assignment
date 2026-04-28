import math
from robot_base import RobotP3DX

class GameState:
    """
    Global state container shared across all robot instances.
    Used to synchronize game phases and trigger cross-robot state transitions.
    """
    first_shot_done = False
    red_goal_scored = False
    shooter_ready_for_pass = False
    pass_done = False
    active_ball_name = 'Bola_Merah' 


class RoleBehaviorMixin:
    """
    Provides core navigation and orientation capabilities for robot movement.
    Designed to be inherited alongside the base hardware class.
    """
    
    def nav_to_point(self, target_pos, stop_dist=0.2, speed_multiplier=1.0, max_speed=5.0):
        """
        Calculates motor velocities required to navigate towards a specific coordinate.
        Automatically reduces base speed during sharp turns to maintain chassis stability.
        """
        pos = self.get_position()
        yaw = self.get_yaw()
        dx = target_pos[0] - pos[0]
        dy = target_pos[1] - pos[1]
        dist = math.hypot(dx, dy)

        if dist < stop_dist:
            return 0.0, 0.0, True 

        target_yaw = math.atan2(dy, dx)
        angle_error = target_yaw - yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        base_speed = 3.5 * speed_multiplier
        turn_gain = 5.0

        # Decrease forward velocity proportionally to the required turn angle
        if speed_multiplier <= 1.0:
            if abs(angle_error) > math.pi / 4:
                base_speed = 0.0
            elif abs(angle_error) > math.pi / 8:
                base_speed *= 0.5

        v_left = base_speed - (turn_gain * angle_error)
        v_right = base_speed + (turn_gain * angle_error)
        
        # Constrain outputs to prevent physics engine instability
        v_left = max(min(v_left, max_speed), -max_speed)
        v_right = max(min(v_right, max_speed), -max_speed)
        
        return v_left, v_right, False

    def face_point(self, target_pos, angle_tol=0.1):
        """
        Calculates differential motor velocities to pivot the robot in place,
        aligning its heading with a target coordinate.
        """
        pos = self.get_position()
        yaw = self.get_yaw()
        dx = target_pos[0] - pos[0]
        dy = target_pos[1] - pos[1]
        
        target_yaw = math.atan2(dy, dx)
        angle_error = target_yaw - yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        if abs(angle_error) < angle_tol:
            return 0.0, 0.0, True 
            
        turn_gain = 4.0
        v_left = -turn_gain * angle_error
        v_right = turn_gain * angle_error
        return v_left, v_right, False


class StrikerRobot(RobotP3DX, RoleBehaviorMixin):
    """
    Primary offensive unit.
    Executes initial strikes, maneuvers to receive passes, and performs secondary strikes.
    Calculates dynamic trajectories to avoid the defending unit.
    """
    
    def __init__(self, sim, robot_name, goal_name, red_ball, blue_ball, gk_name):
        super().__init__(sim, robot_name)
        self.goal_handle = self.sim.getObject(f'/{goal_name}')
        self.red_ball = self.sim.getObject(f'/{red_ball}')
        self.blue_ball = self.sim.getObject(f'/{blue_ball}')
        self.gk_handle = self.sim.getObject(f'/{gk_name}')
        
        self.state = "TARGET_RED"
        self.receive_pos = [-1.5, 0.0]
        self.center_pos = [0.0, 0.0]
        self.kicked_red = False
        self.kicked_blue = False
        self.kick_start_time = 0.0
        self.current_aim_pos = [0.0, 0.0]

    def get_smart_aim(self):
        """
        Determines the optimal target vector based on the defender's current position,
        aiming for the unguarded section of the goal.
        """
        goal_pos = self.sim.getObjectPosition(self.goal_handle, self.sim.handle_world)
        gk_pos = self.sim.getObjectPosition(self.gk_handle, self.sim.handle_world)
        
        # Calculate lateral offset opposite to the goalkeeper's position
        offset_y = -0.8 if gk_pos[1] > 0.0 else 0.8
        
        # Set target depth beyond the goal line to ensure successful entry
        return [goal_pos[0] + 1.0, 0.0 + offset_y]

    def step(self):
        """Executes the state machine logic for offensive maneuvers."""
        pos = self.get_position()
        goal_pos = self.sim.getObjectPosition(self.goal_handle, self.sim.handle_world)
        red_pos = self.sim.getObjectPosition(self.red_ball, self.sim.handle_world)
        blue_pos = self.sim.getObjectPosition(self.blue_ball, self.sim.handle_world)

        if self.state == "TARGET_RED":
            dx = red_pos[0] - goal_pos[0]
            dy = red_pos[1] - goal_pos[1]
            dist = math.hypot(dx, dy)
            if dist > 0:
                standby = [red_pos[0] + (dx/dist)*0.8, red_pos[1] + (dy/dist)*0.8]
                vl, vr, reached = self.nav_to_point(standby, stop_dist=0.2)
                self.set_velocities(vl, vr)
                if reached:
                    self.state = "AIM_RED"

        elif self.state == "AIM_RED":
            self.current_aim_pos = self.get_smart_aim()
            vl, vr, aligned = self.face_point(self.current_aim_pos)
            self.set_velocities(vl, vr)
            if aligned:
                self.state = "SHOOT_RED"

        elif self.state == "SHOOT_RED":
            dist_to_ball = math.hypot(pos[0]-red_pos[0], pos[1]-red_pos[1])
            
            if not self.kicked_red:
                if dist_to_ball < 0.7:
                    self.kicked_red = True
                    self.kick_start_time = self.sim.getSimulationTime()
                else:
                    vl, vr, _ = self.nav_to_point(red_pos, stop_dist=0.0)
                    self.set_velocities(vl, vr)
            
            if self.kicked_red:
                if self.sim.getSimulationTime() - self.kick_start_time < 0.5:
                    vl, vr, _ = self.nav_to_point(self.current_aim_pos, stop_dist=0.0, speed_multiplier=3.0, max_speed=15.0)
                    self.set_velocities(vl, vr)
                else:
                    self.set_velocities(0, 0) # Brake
                    if dist_to_ball > 1.0 or (self.sim.getSimulationTime() - self.kick_start_time > 2.0):
                        GameState.first_shot_done = True
                        self.state = "WAIT_FOR_GOAL"

        elif self.state == "WAIT_FOR_GOAL":
            self.set_velocities(0, 0)
            # Only start moving for the next pass once the red ball actually scores
            if GameState.red_goal_scored:
                self.state = "MOVE_RECEIVE"

        elif self.state == "MOVE_RECEIVE":
            vl, vr, reached = self.nav_to_point(self.receive_pos)
            self.set_velocities(vl, vr)
            if reached:
                GameState.shooter_ready_for_pass = True
                self.state = "WAIT_PASS"

        elif self.state == "WAIT_PASS":
            vl, vr, _ = self.face_point(blue_pos)
            self.set_velocities(vl, vr)
            if GameState.pass_done:
                self.kick_start_time = self.sim.getSimulationTime()
                self.state = "INTERCEPT_BLUE"

        elif self.state == "INTERCEPT_BLUE":
            self.set_velocities(0, 0)
            dist_to_ball = math.hypot(pos[0]-blue_pos[0], pos[1]-blue_pos[1])
            
            # Maintain stationary position until ball enters threshold or timeout is reached
            if dist_to_ball < 0.5 or (self.sim.getSimulationTime() - self.kick_start_time > 3.5):
                self.state = "NAV_AROUND_BLUE"

        elif self.state == "NAV_AROUND_BLUE":
            # Go to a safe waypoint 1.5m to the side of the blue ball
            safe_waypoint = [blue_pos[0], blue_pos[1] + 1.5]
            vl, vr, reached = self.nav_to_point(safe_waypoint, stop_dist=0.4)
            self.set_velocities(vl, vr)
            if reached:
                self.state = "POSITION_BLUE"

        elif self.state == "POSITION_BLUE":
            # Get behind the blue ball
            dx = blue_pos[0] - goal_pos[0]
            dy = blue_pos[1] - 0.0
            dist = math.hypot(dx, dy)
            if dist > 0:
                standby = [blue_pos[0] + (dx/dist)*0.8, blue_pos[1] + (dy/dist)*0.8]
                vl, vr, reached = self.nav_to_point(standby, stop_dist=0.2)
                self.set_velocities(vl, vr)
                if reached:
                    self.state = "AIM_BLUE"

        elif self.state == "AIM_BLUE":
            self.current_aim_pos = self.get_smart_aim()
            vl, vr, aligned = self.face_point(self.current_aim_pos)
            self.set_velocities(vl, vr)
            if aligned:
                self.state = "SHOOT_BLUE"

        elif self.state == "SHOOT_BLUE":
            dist_to_ball = math.hypot(pos[0]-blue_pos[0], pos[1]-blue_pos[1])
            
            if not self.kicked_blue:
                if dist_to_ball < 0.7:
                    self.kicked_blue = True
                    self.kick_start_time = self.sim.getSimulationTime()
                else:
                    vl, vr, _ = self.nav_to_point(blue_pos, stop_dist=0.0)
                    self.set_velocities(vl, vr)
                
            if self.kicked_blue:
                if self.sim.getSimulationTime() - self.kick_start_time < 0.5:
                    vl, vr, _ = self.nav_to_point(self.current_aim_pos, stop_dist=0.0, speed_multiplier=5.0, max_speed=50.0)
                    self.set_velocities(vl, vr)
                else:
                    self.set_velocities(0, 0)
                    if dist_to_ball > 1.0 or (self.sim.getSimulationTime() - self.kick_start_time > 2.0):
                        self.state = "DONE"
                
        elif self.state == "DONE":
            self.set_velocities(0, 0)


class PasserRobot(RobotP3DX, RoleBehaviorMixin):
    """
    Support unit.
    Safely approaches the secondary ball and executes a controlled pass to the primary unit.
    """

    def __init__(self, sim, robot_name, blue_ball):
        super().__init__(sim, robot_name)
        self.blue_ball = self.sim.getObject(f'/{blue_ball}')
        self.state = "NAV_AROUND_BALL"
        self.receive_pos = [-1.5, 0.0]
        self.pass_attempted = False
        self.kick_start_time = 0.0

    def step(self):
        """Executes the state machine logic for passing maneuvers."""
        pos = self.get_position()
        blue_pos = self.sim.getObjectPosition(self.blue_ball, self.sim.handle_world)

        if self.state == "NAV_AROUND_BALL":
            # Target a lateral safe zone to prevent premature collision
            safe_waypoint = [blue_pos[0], blue_pos[1] + 1.5]
            vl, vr, reached = self.nav_to_point(safe_waypoint, stop_dist=0.4)
            self.set_velocities(vl, vr)
            if reached:
                self.state = "POSITIONING"

        elif self.state == "POSITIONING":
            dx = blue_pos[0] - self.receive_pos[0]
            dy = blue_pos[1] - self.receive_pos[1]
            dist = math.hypot(dx, dy)
            if dist > 0:
                standby = [blue_pos[0] + (dx/dist)*1.0, blue_pos[1] + (dy/dist)*1.0]
                
                vl, vr, reached = self.nav_to_point(standby, stop_dist=0.3)
                self.set_velocities(vl, vr)
                if reached:
                    self.state = "WAITING"

        elif self.state == "WAITING":
            vl, vr, aligned = self.face_point(blue_pos)
            self.set_velocities(vl, vr)
            
            # Delay execution until primary unit is ready and prior game phase completes
            if GameState.shooter_ready_for_pass and GameState.red_goal_scored and aligned:
                self.state = "PASSING"

        elif self.state == "PASSING":
            dist_to_ball = math.hypot(pos[0]-blue_pos[0], pos[1]-blue_pos[1])
            
            if not self.pass_attempted:
                if dist_to_ball < 0.7:
                    self.pass_attempted = True
                    self.kick_start_time = self.sim.getSimulationTime()
                else:
                    vl, vr, _ = self.nav_to_point(blue_pos, stop_dist=0.0)
                    self.set_velocities(vl, vr)
            
            if self.pass_attempted:
                # Apply short burst of velocity for controlled pass transmission
                if self.sim.getSimulationTime() - self.kick_start_time < 0.5:
                    vl, vr, _ = self.nav_to_point(self.receive_pos, stop_dist=0.0, speed_multiplier=2.0, max_speed=15.0)
                    self.set_velocities(vl, vr)
                else:
                    self.set_velocities(0, 0)
                    if dist_to_ball > 1.0 or (self.sim.getSimulationTime() - self.kick_start_time > 2.0):
                        GameState.pass_done = True
                        self.state = "DONE"

        elif self.state == "DONE":
            self.set_velocities(0, 0)


class GoalkeeperRobot(RobotP3DX, RoleBehaviorMixin):
    """
    Defensive unit.
    Performs baseline area denial until an active threat is detected, then tracks and intercepts.
    """
    
    def __init__(self, sim, robot_name, goal_name):
        super().__init__(sim, robot_name)
        self.goal_handle = self.sim.getObject(f'/{goal_name}')
        self.state = "DUMB_PATROL"
        self.start_time = self.sim.getSimulationTime()

    def step(self):
        """Executes the state machine logic for defensive maneuvering and interception."""
        pos = self.get_position()
        goal_pos = self.sim.getObjectPosition(self.goal_handle, self.sim.handle_world)
        t = self.sim.getSimulationTime() - self.start_time

        if self.state == "DUMB_PATROL":
            # Establish baseline patrol trajectory covering the goal width
            target_x = goal_pos[0] - 0.5  
            target_y = 0.0 + math.sin(t * 1.2) * 1.1 
            
            vl, vr, _ = self.nav_to_point([target_x, target_y], stop_dist=0.1)
            self.set_velocities(vl, vr)

            if GameState.red_goal_scored:
                self.state = "SMART_GUARD"

        elif self.state in ["SMART_GUARD", "CHASE_BALL"]:
            active_ball_handle = self.sim.getObject(f'/{GameState.active_ball_name}')
            ball_pos = self.sim.getObjectPosition(active_ball_handle, self.sim.handle_world)
            dist_to_goal = math.hypot(ball_pos[0]-goal_pos[0], ball_pos[1]-goal_pos[1])

            # Trigger active interception if threat enters defense radius
            if dist_to_goal < 3.0:
                self.state = "CHASE_BALL"
            else:
                self.state = "SMART_GUARD"

            if self.state == "CHASE_BALL":
                vl, vr, _ = self.nav_to_point(ball_pos, stop_dist=0.2)
                self.set_velocities(vl, vr)
            else:
                # Maintain defensive line matching the lateral position of the threat
                target_x = goal_pos[0] - 0.5
                target_y = max(min(ball_pos[1], 1.1), -1.1)
                vl, vr, _ = self.nav_to_point([target_x, target_y], stop_dist=0.1)
                self.set_velocities(vl, vr)