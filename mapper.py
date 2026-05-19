import numpy as np

class OccupancyGridMap:
    def __init__(self, width_m=9.0, height_m=18.0, resolution=0.1):
        """
        Initializes the grid map.
        Assumes the CoppeliaSim world origin (0,0) is in the center of the room.
            width_m, height_m: Physical dimensions of the map in meters.
            resolution: Size of each grid cell in meters (e.g., 0.1m means each cell is 10cm x 10cm).
        """
        self.resolution = resolution
        self.width_cells = int(width_m / resolution)
        self.height_cells = int(height_m / resolution)
        
        # Log-odds probabilities
        # 0 means unknown. Positive means obstacle, negative means free space.
        self.grid = np.zeros((self.width_cells, self.height_cells), dtype=np.float32)
        
        # Tuning parameters for sensor confidence
        self.l_occ = 0.85   # How confident when sensor hits something
        self.l_free = -0.4  # How confident when sensor sees empty space
        self.max_log_odd = 10.0
        self.min_log_odd = -10.0
        
        # Offset to map world center (0,0) to grid center (width/2, height/2)
        self.offset_x = width_m / 2.0
        self.offset_y = height_m / 2.0

    def world_to_grid(self, x_world, y_world):
        """
        Converts world coordinates (meters) to grid indices (integers) based on the map's resolution and offset.
            x_world, y_world: Real-world coordinates in meters.
        Returns:
            x_grid, y_grid: Corresponding grid indices.
        """
        x_grid = int((x_world + self.offset_x) / self.resolution)
        y_grid = int((y_world + self.offset_y) / self.resolution)
        
        # Keep indices within bounds
        x_grid = max(0, min(x_grid, self.width_cells - 1))
        y_grid = max(0, min(y_grid, self.height_cells - 1))
        return x_grid, y_grid

    def bresenham_line(self, x0, y0, x1, y1):
        """
        Standard ray-tracing algorithm. 
        Finds all grid cells that a sensor beam passes through.
            x0, y0: Starting grid cell (robot position)
            x1, y1: Ending grid cell (sensor hit position)
        Returns a list of (x_grid, y_grid) tuples for each cell the beam traverses.
        """
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x, y))
        return cells

    def update_map(self, robot_x, robot_y, sensor_hit_x, sensor_hit_y, hit_obstacle):
        """
        Updates the map based on a single sensor reading.
            robot_x, robot_y: Robot's current position in world coordinates (meters).
            sensor_hit_x, sensor_hit_y: The point where the sensor beam ended (either hit or max range) in world coordinates (meters).
            hit_obstacle: Boolean (True if sensor hit something, False if it reached max range without hitting)
        """
        # Convert real-world coordinates to grid coordinates
        rx_grid, ry_grid = self.world_to_grid(robot_x, robot_y)
        sx_grid, sy_grid = self.world_to_grid(sensor_hit_x, sensor_hit_y)
        
        # Get all cells the sensor beam passed through
        ray_cells = self.bresenham_line(rx_grid, ry_grid, sx_grid, sy_grid)
        
        # Update probabilities
        for i, (gx, gy) in enumerate(ray_cells):
            if i == len(ray_cells) - 1 and hit_obstacle:
                # The final cell is an obstacle
                self.grid[gx, gy] += self.l_occ
            else:
                # The beam passed through this cell, so it is free
                self.grid[gx, gy] += self.l_free
                
            # Clamp values to prevent numerical overflow
            self.grid[gx, gy] = np.clip(self.grid[gx, gy], self.min_log_odd, self.max_log_odd)

    def get_binary_map(self):
        """
        Returns a strict binary map. 
        Threshold > 1.5 requires at least 2 consecutive sensor hits 
        to confirm an obstacle, filtering out sensor noise/ghosts.

        Returns:
            A 2D numpy array of uint8 where:
                0 = unknown or free space
                1 = occupied (obstacle)
        """
        return (self.grid > 1.5).astype(np.uint8)