import numpy as np
import heapq
import math

class Node:
    def __init__(self, parent=None, position=None):
        """
        Init a node for A* pathfinding.
            parent: Node that led to this one (for path reconstruction)
            position: (x, y) grid coordinates   
        """
        self.parent = parent
        self.position = position

        self.g = 0 
        self.h = 0 
        self.f = 0 

    def __eq__(self, other):
        """Nodes are equal if they occupy the same grid cell."""
        return self.position == other.position
    
    def __lt__(self, other):
        """For heapq to compare nodes based on f-cost."""
        return self.f < other.f

class AStarPlanner:
    def __init__(self, resolution=0.1, lethal_radius_m=0.25, risk_radius_m=0.45):
        """
        A* path planner that generates a gradient costmap from a binary occupancy grid.
            resolution: meters per grid cell
            lethal_radius_m: radius around obstacles that is considered a collision (robot's physical size)
            risk_radius_m: additional radius around obstacles that is considered risky (safety buffer)
        """
        self.resolution = resolution
        # Lethal: The physical size of the robot. Touching this means a crash.
        self.lethal_cells = int(math.ceil(lethal_radius_m / resolution))
        # Risk: The safety padding. We prefer not to be here, but we will if forced.
        self.risk_cells = int(math.ceil(risk_radius_m / resolution))

    def inflate_map(self, binary_map):
        """
        Generates a Gradient Costmap instead of a binary wall map.
        - Cells with value 0.0 are free and safe.
        - Cells with value 100.0 are impassable (lethal).
        - Cells with values between 0.0 and 100.0 represent risk levels (the closer to 100, the more dangerous).

            binary_map: 2D numpy array where 0=free, 1=obstacle
        Returns a 2D numpy array of the same shape with inflated costs.
        """
        
        costmap = np.zeros_like(binary_map, dtype=np.float32)
        rows, cols = binary_map.shape
        obs_x, obs_y = np.where(binary_map == 1)

        for ox, oy in zip(obs_x, obs_y):
            # Limit the search box for speed
            min_i = max(-self.risk_cells, -ox)
            max_i = min(self.risk_cells + 1, rows - ox)
            min_j = max(-self.risk_cells, -oy)
            max_j = min(self.risk_cells + 1, cols - oy)
            
            for i in range(min_i, max_i):
                for j in range(min_j, max_j):
                    dist = math.hypot(i, j)
                    if dist <= self.risk_cells:
                        nx, ny = ox + i, oy + j
                        
                        # If it's physically a wall or inside the robot's lethal radius
                        if dist <= self.lethal_cells:
                            costmap[nx, ny] = 100.0 # 100 is impassable
                        else:
                            # It's in the risk zone. Calculate a penalty.
                            # Closer to the wall = higher penalty (e.g., 50 down to 1)
                            if costmap[nx, ny] != 100.0: # Don't overwrite lethal cells
                                risk_score = 40.0 * (1.0 - (dist - self.lethal_cells) / (self.risk_cells - self.lethal_cells))
                                costmap[nx, ny] = max(costmap[nx, ny], risk_score)
                                
        return costmap

    def plan(self, binary_map, start_idx, goal_idx):
        """
        Plans a path from start_idx to goal_idx on the given binary_map using A* with the inflated costmap.
            binary_map: 2D numpy array where 0=free, 1=obstacle
            start_idx: (x, y) grid coordinates of the start
            goal_idx: (x, y) grid coordinates of the goal
        Returns a list of grid indices from start to goal, or None if no path is found
        """
        # Prepare the Costmap
        grid = self.inflate_map(binary_map)
        max_x, max_y = grid.shape

        # Clear the robot's immediate start zone so it can escape tight spots
        safe_radius = self.lethal_cells
        min_x = max(0, start_idx[0] - safe_radius)
        max_x_bound = min(max_x, start_idx[0] + safe_radius + 1)
        min_y = max(0, start_idx[1] - safe_radius)
        max_y_bound = min(max_y, start_idx[1] + safe_radius + 1)
        
        for x in range(min_x, max_x_bound):
            for y in range(min_y, max_y_bound):
                if binary_map[x, y] == 0: 
                    grid[x, y] = 0.0 # Clear lethal/risk flags under the wheels

        # Check bounds
        if grid[start_idx[0], start_idx[1]] == 100.0:
            print("A* Error: Start position is strictly inside a REAL obstacle!")
            return None
        if grid[goal_idx[0], goal_idx[1]] == 100.0:
            print("A* Error: Goal position is strictly inside an obstacle!")
            return None

        # Standard A* Setup
        start_node = Node(None, start_idx)
        goal_node = Node(None, goal_idx)
        open_list = []
        closed_set = set()
        heapq.heappush(open_list, start_node)
        g_costs = {start_idx: 0}

        movements = [
            (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
            (1, 1, 1.414), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414)
        ]

        while len(open_list) > 0:
            current_node = heapq.heappop(open_list)
            closed_set.add(current_node.position)

            if current_node == goal_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]

            for dx, dy, cost in movements:
                node_position = (current_node.position[0] + dx, current_node.position[1] + dy)

                if not (0 <= node_position[0] < max_x and 0 <= node_position[1] < max_y):
                    continue

                # Impassable Wall Check (Only reject if it is exactly 100.0)
                cell_penalty = grid[node_position[0], node_position[1]]
                if cell_penalty == 100.0:
                    continue

                if node_position in closed_set:
                    continue

                # Add the costmap penalty to the distance cost.
                # If a cell is risky (e.g. penalty 30), A* will try to route around it.
                # But if there is no other way, it will swallow the cost and squeeze through.
                new_g = current_node.g + cost + (cell_penalty * 2.0) 

                if node_position not in g_costs or new_g < g_costs[node_position]:
                    g_costs[node_position] = new_g
                    
                    new_node = Node(current_node, node_position)
                    new_node.g = new_g
                    new_node.h = math.hypot(node_position[0] - goal_node.position[0], 
                                            node_position[1] - goal_node.position[1])
                    new_node.f = new_node.g + new_node.h

                    heapq.heappush(open_list, new_node)

        print("A* Error: No valid path found to goal.")
        return None