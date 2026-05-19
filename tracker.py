import math

class PurePursuitTracker:
    def __init__(self, lookahead_distance=0.4, base_speed=0.3, stop_tolerance=0.15):
        """
        A simple Pure Pursuit tracker for differential drive robots.
            lookahead_distance: How far ahead on the path to look for the target point.
            base_speed: The forward speed when moving towards the target.
            stop_tolerance: Distance to final goal at which to stop.
        """
        self.Ld = lookahead_distance
        self.base_speed = base_speed
        self.stop_tolerance = stop_tolerance
        self.path = []
        self.closest_idx = 0

    def set_path(self, world_path):
        """
        Loads a new path to track and resets index.
            world_path: List of (x, y) coordinates in the world frame.
        """
        self.path = world_path
        self.closest_idx = 0

    def get_remaining_path(self):
        """
        Returns only the path in front of the robot.
        Returns a list of (x, y) coordinates that are still ahead of the robot.
        """
        if not self.path: return []
        return self.path[self.closest_idx:]

    def compute_velocities(self, robot_x, robot_y, robot_yaw):
        """
        Computes the linear and angular velocities to follow the path.
            robot_x, robot_y: Current position of the robot in world coordinates.
            robot_yaw: Current orientation of the robot in radians (0 facing right, counterclockwise positive)
        Returns (v, w, done) where v is linear velocity, w is angular velocity, and done is True if the goal is reached.
        """
        if not self.path:
            return 0.0, 0.0, True 

        final_goal = self.path[-1]
        dist_to_final = math.hypot(final_goal[0] - robot_x, final_goal[1] - robot_y)
        
        # Stop Condition
        if dist_to_final < self.stop_tolerance:
            self.path = []
            return 0.0, 0.0, True

        # Find the closest point
        min_dist = float('inf')
        for i in range(self.closest_idx, len(self.path)):
            d = math.hypot(self.path[i][0] - robot_x, self.path[i][1] - robot_y)
            if d < min_dist:
                min_dist = d
                self.closest_idx = i

        # Find Look-Ahead Point
        target_point = final_goal
        for i in range(self.closest_idx, len(self.path)):
            d = math.hypot(self.path[i][0] - robot_x, self.path[i][1] - robot_y)
            if d >= self.Ld:
                target_point = self.path[i]
                break

        # Local Frame Transformation
        dx = target_point[0] - robot_x
        dy = target_point[1] - robot_y
        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)

        dist_to_target = math.hypot(local_x, local_y)
        if dist_to_target < 0.001:
            return 0.0, 0.0, False

        # Point turn 
        # Calculate the angle to the target (-pi to pi)
        angle_to_target = math.atan2(local_y, local_x)
        
        # If the target is more than ~35 degrees to the side, spin in place
        if abs(angle_to_target) > 0.6: 
            v = 0.0
            # Proportional rotation speed, capped at 1.0 rad/s
            w = 1.5 * angle_to_target 
            w = max(-1.0, min(1.0, w))
            return v, w, False

        # Normal Pure Pursuit once robot is roughly facing the target
        gamma = (2 * local_y) / (dist_to_target**2)

        v = self.base_speed
        if abs(gamma) > 1.5: 
            v = 0.1 
        
        if dist_to_final < self.Ld:
            v = max(0.05, self.base_speed * (dist_to_final / self.Ld))

        w = v * gamma
        w = max(-1.0, min(1.0, w))

        return v, w, False