import os
import io
import ast
import heapq
import sys
import time
import math
import json
import random
import threading

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from dotenv import load_dotenv

from google import genai
from google.genai import types
import ollama

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

load_dotenv()

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

class GeminiNavigator:
    def __init__(self, config_file="report/config.json", memory_file="memory.json"):
        # Initialize the modern SDK client (Automatically picks up GEMINI_API_KEY from env)
        try:
            self.client = genai.Client()
        except Exception as e:
            print(f"[ERROR] Failed to init GenAI Client. Did you set GEMINI_API_KEY? Error: {e}")
            
        self.memory_file = memory_file
        self.model_name = 'gemma-4-31b-it' # 'gemma-4-26b-a4b-it'
        self.last_target_object = None # Store the clean object name for verification
        
        # Load Prompts
        with open(config_file, 'r') as f:
            self.config = json.load(f)

    def _load_memory(self):
        if not os.path.exists(self.memory_file):
            return {"spatial_memory": []}
        try:
            with open(self.memory_file, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError:
            return {"spatial_memory": []}

    def _save_memory(self, kb):
        with open(self.memory_file, 'w') as f:
            json.dump(kb, f, indent=2)

    def analyze_exploration_snapshot(self, pil_image, grid_idx, yaw):
        """Phase 1: Looks at an image, extracts landmarks, updates JSON."""
        prompt = f"{self.config['system_context']} \nTask: {self.config['exploration_prompt']}"

        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[pil_image, prompt]
            )
            description = response.text.strip()
            
            kb = self._load_memory()
            kb["spatial_memory"].append({"grid_idx": grid_idx, "yaw": yaw, "view": description})
            self._save_memory(kb)
            
            return description
        except Exception as e:
            return f"Error connecting to Gemini: {e}"

    def remove_invalid_memory(self, grid_idx):
        """Phase 2: If target is no longer there, delete the memory entry."""
        kb = self._load_memory()
        kb["spatial_memory"] = [m for m in kb["spatial_memory"] if m["grid_idx"] != grid_idx]
        self._save_memory(kb)

    def get_execution_target(self, user_command):
        """Phase 2: Reads user command + JSON memory to determine target coordinates."""
        kb = self._load_memory()
        
        prompt = f"""
        {self.config['system_context']}
        
        User Command: "{user_command}"
        Spatial Memory Database: {json.dumps(kb)}
        
        Task:
        1. Identify the user's intent. If they are just chatting and NOT asking to navigate/search, set status to NOT_A_COMMAND.
        2. Extract the exact core object the user wants (e.g. if command is "move closer to the laptop", target_object is "laptop").
        3. If the object IS in the memory database, return FOUND_IN_MEMORY and its coords/yaw.
        4. If the object IS NOT in the database, return SEARCH_REQUESTED.
        """
        
        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=prompt,
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_schema={
                        "type": "OBJECT",
                        "properties": {
                            "status": {"type": "STRING", "enum": ["FOUND_IN_MEMORY", "SEARCH_REQUESTED", "NOT_A_COMMAND"]},
                            "target_object": {"type": "STRING", "description": "The exact physical object isolated from the prompt"},
                            "coords": {"type": "ARRAY", "items": {"type": "INTEGER"}, "nullable": True},
                            "yaw": {"type": "NUMBER", "nullable": True},
                            "reasoning": {"type": "STRING"}
                        },
                        "required": ["status", "target_object", "reasoning"]
                    }
                )
            )
            
            result = json.loads(response.text)
            
            # Save the clean object name so the verification step can use it later
            if result.get("target_object"):
                self.last_target_object = result["target_object"]
            else:
                self.last_target_object = user_command
                
            return result
        except Exception as e:
            return {"status": "NOT_A_COMMAND", "coords": None, "reasoning": f"API Error: {e}"}

    def verify_target_presence(self, pil_image, target_name):
        """Phase 2: Validates if the target actually exists at the coordinates."""
        # Use the cleanly extracted object name instead of the raw user prompt
        actual_target = getattr(self, 'last_target_object', target_name)
        
        prompt = self.config['verification_prompt'].replace("{target}", actual_target)
        
        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[pil_image, prompt]
            )
            return "YES" in response.text.upper()
        except Exception:
            return False

class OllamaNavigator:
    def __init__(self, config_file="report/config.json", memory_file="memory.json", model_name="gemma4:e2b"):
        self.memory_file = memory_file
        self.model_name = model_name 
        self.last_target_object = None # Store the clean object name for verification
        
        # Load Prompts
        with open(config_file, 'r') as f:
            self.config = json.load(f)
            
        print(f"[SYSTEM] Initialized Local Ollama Agent using model: {self.model_name}")

    def _load_memory(self):
        if not os.path.exists(self.memory_file):
            return {"spatial_memory": []}
        try:
            with open(self.memory_file, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError:
            return {"spatial_memory": []}

    def _save_memory(self, kb):
        with open(self.memory_file, 'w') as f:
            json.dump(kb, f, indent=2)

    def _pil_to_bytes(self, pil_image):
        """Ollama requires raw image bytes."""
        byte_stream = io.BytesIO()
        # Convert to RGB just in case it's RGBA to avoid JPEG errors
        if pil_image.mode != 'RGB':
            pil_image = pil_image.convert('RGB')
        pil_image.save(byte_stream, format='JPEG')
        return byte_stream.getvalue()

    def analyze_exploration_snapshot(self, pil_image, grid_idx, yaw):
        """Phase 1: Looks at an image, extracts landmarks, updates JSON."""
        prompt = f"{self.config['system_context']} \nTask: {self.config['exploration_prompt']}\nReply with just one short sentence."
        
        try:
            response = ollama.chat(
                model=self.model_name,
                messages=[{
                    'role': 'user',
                    'content': prompt,
                    'images': [self._pil_to_bytes(pil_image)]
                }]
            )
            description = response['message']['content'].strip()
            
            kb = self._load_memory()
            kb["spatial_memory"].append({"grid_idx": list(grid_idx), "yaw": float(yaw), "view": description})
            self._save_memory(kb)
            
            return description
        except Exception as e:
            return f"Error connecting to Ollama: {e}"

    def remove_invalid_memory(self, grid_idx):
        """Phase 2: If target is no longer there, delete the memory entry."""
        kb = self._load_memory()
        kb["spatial_memory"] = [m for m in kb["spatial_memory"] if m["grid_idx"] != list(grid_idx)]
        self._save_memory(kb)

    def get_execution_target(self, user_command):
        """Phase 2: Reads user command + JSON memory to determine target coordinates."""
        kb = self._load_memory()
        
        prompt = f"""
        {self.config['system_context']}
        
        User Command: "{user_command}"
        Spatial Memory Database: {json.dumps(kb)}
        
        Task:
        1. Identify the user's intent. If they are just chatting and NOT asking to navigate/search, set status to NOT_A_COMMAND.
        2. Extract the exact core object the user wants (e.g. if command is "move closer to the laptop", target_object is "laptop").
        3. If the object IS in the memory database, return FOUND_IN_MEMORY and its coords/yaw.
        4. If the object IS NOT in the database, return SEARCH_REQUESTED.
        
        Output ONLY a valid JSON object matching this exact schema. 
        CRITICAL RULES:
        - Do NOT wrap the JSON in markdown blocks or backticks.
        - You MUST use DOUBLE QUOTES ("") for all property names and string values. Never use single quotes ('').
        
        {{
            "status": "FOUND_IN_MEMORY" or "SEARCH_REQUESTED" or "NOT_A_COMMAND",
            "target_object": "string (the physical object)",
            "coords": [x, y] or null,
            "yaw": float or null,
            "reasoning": "string explaining your choice"
        }}
        """
        
        try:
            # Enforce JSON format natively through Ollama
            response = ollama.chat(
                model=self.model_name,
                messages=[{
                    'role': 'user',
                    'content': prompt
                }],
                format='json'
            )
            
            raw_content = response['message']['content'].strip()
            
            # Isolate the JSON block (ignores conversational babble before/after)
            start_idx = raw_content.find('{')
            end_idx = raw_content.rfind('}')
            
            if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
                raw_content = raw_content[start_idx:end_idx+1]
            
            try:
                # Try standard strict JSON parsing
                result = json.loads(raw_content)
            except json.JSONDecodeError as e:
                # Fallback: If the model stubbornly used single quotes, it's a Python dict format.
                # json.loads() fails on single quotes, but ast.literal_eval parses them perfectly.
                try:
                    python_str = raw_content.replace('null', 'None').replace('true', 'True').replace('false', 'False')
                    result = ast.literal_eval(python_str)
                except Exception:
                    # If both parsers fail, raise the original JSON error to be caught by the outer block
                    raise e
            
            # Store the clean object name
            if result.get("target_object"):
                self.last_target_object = result["target_object"]
            else:
                self.last_target_object = user_command
                
            return result
            
        except Exception as e:
            return {"status": "NOT_A_COMMAND", "coords": None, "reasoning": f"Local Inference Error: {e}"}

    def verify_target_presence(self, pil_image, target_name):
        """Phase 2: Validates if the target actually exists at the coordinates."""
        # Use the cleanly extracted object name instead of the raw user prompt
        actual_target = getattr(self, 'last_target_object', target_name)
        if not actual_target:
            actual_target = target_name
            
        prompt = self.config['verification_prompt'].replace("{target}", actual_target)
        
        try:
            response = ollama.chat(
                model=self.model_name,
                messages=[{
                    'role': 'user',
                    'content': prompt,
                    'images': [self._pil_to_bytes(pil_image)]
                }]
            )
            return "YES" in response['message']['content'].upper()
        except Exception:
            return False

# Load configuration rules
with open("report/config.json", "r") as f:
    app_config = json.load(f)

EXPLORATION_TARGET = tuple(app_config.get("exploration_target_grid", [250, 250]))
SNAPSHOT_DISTANCE = app_config.get("snapshot_distance_m", 2.0)

def grid_to_world(x_grid, y_grid, map_obj):
    x_world = (x_grid * map_obj.resolution) - map_obj.offset_x
    y_world = (y_grid * map_obj.resolution) - map_obj.offset_y
    return x_world, y_world

def capture_image(sim_client, handle):
    img, res = sim_client.getVisionSensorImg(handle)
    img_np = np.frombuffer(img, dtype=np.uint8).reshape((res[1], res[0], 3))
    img_np = np.flipud(img_np) 
    return Image.fromarray(img_np)

def get_frontier_cells(grid_map):
    """
    Finds the boundary where Free Space touches Unknown Space.
    Used as the secondary 'deep search' tool when the dummy point fails.
    """
    free_mask = grid_map < -0.2
    unknown_mask = (grid_map >= -0.1) & (grid_map <= 0.1)
    
    # Check adjacency
    up = np.roll(unknown_mask, 1, axis=0)
    down = np.roll(unknown_mask, -1, axis=0)
    left = np.roll(unknown_mask, 1, axis=1)
    right = np.roll(unknown_mask, -1, axis=1)
    
    frontier_mask = free_mask & (up | down | left | right)
    
    # Clear borders
    frontier_mask[0, :] = False
    frontier_mask[-1, :] = False
    frontier_mask[:, 0] = False
    frontier_mask[:, -1] = False
    
    return np.argwhere(frontier_mask)

def get_nearest_safe_cell(binary_map, planner_obj, target_idx, search_radius=12):
    costmap = planner_obj.inflate_map(binary_map)
    tx, ty = target_idx
    
    if costmap[tx, ty] < 90.0:
        return (tx, ty)
        
    best_cell = None
    min_dist = float('inf')
    
    min_x = max(0, tx - search_radius)
    max_x = min(costmap.shape[0], tx + search_radius + 1)
    min_y = max(0, ty - search_radius)
    max_y = min(costmap.shape[1], ty + search_radius + 1)
    
    for x in range(min_x, max_x):
        for y in range(min_y, max_y):
            if costmap[x, y] < 90.0:
                dist = math.hypot(x - tx, y - ty)
                if dist < min_dist:
                    min_dist = dist
                    best_cell = (x, y)
                    
    return best_cell

print("[SYSTEM] Initializing CoppeliaSim Connection...")
client = RemoteAPIClient()
sim = client.require('sim')
client.setStepping(True) 
sim.startSimulation()

# Hardware Handles
p3dx = sim.getObject("/PioneerP3DX")
p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")
vision_sensor = sim.getObject("/PioneerP3DX/visionSensor")

if sim.getObjectType(vision_sensor) != sim.object_visionsensor_type:
    print("\n[CRITICAL ERROR] The object at '/PioneerP3DX/visionSensor' is NOT a Vision Sensor.")
    sim.stopSimulation()
    sys.exit(1)

sensor_handles = [sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]") for i in range(9)]

# AI & Nav Instances 
my_map = OccupancyGridMap(width_m=30.0, height_m=30.0, resolution=0.1)
my_map.l_free = -0.5
planner = AStarPlanner(resolution=0.1, lethal_radius_m=0.3, risk_radius_m=0.5)
tracker = PurePursuitTracker(lookahead_distance=0.4, base_speed=0.25, stop_tolerance=0.15)
agent = GeminiNavigator()
# agent = OllamaNavigator()

# State Machine Initialization
state = "EXPLORE_PLANNING"
world_path = []
active_target_idx = None
active_target_yaw = None
active_search_target = None  

last_snap_pos = None
last_snap_ori = None
snapshot_markers = []
verification_markers = [] 
current_target_name = ""

last_pos = (0, 0)
last_pos_time = time.time()
loop_counter = 0

# GUI Setup
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))

def on_key(event):
    global state, active_search_target
    if event.key == 'enter' and "EXPLORE" in state:
        print("\n[PHASE SHIFT] Exploration Terminated. Entering Phase 2 (Execution)...")
        sim.setJointTargetVelocity(p3dx_lw, 0)
        sim.setJointTargetVelocity(p3dx_rw, 0)
        active_search_target = None 
        state = "EXECUTION_IDLE"

fig.canvas.mpl_connect('key_press_event', on_key)

im = ax.imshow(my_map.grid.T, cmap='Greys', origin='lower', vmin=-3, vmax=3)
path_line, = ax.plot([], [], 'r-', linewidth=2, label="A* Path")
robot_dot, = ax.plot([], [], 'bo', markersize=6, label="Robot")
memory_stars, = ax.plot([], [], 'y*', markersize=10, label="Memory Logged")
verify_dots, = ax.plot([], [], 'go', markersize=8, label="Verified Target")
ax.set_title("Semantic SLAM (Press ENTER to switch to Chatbot)")
plt.legend()
plt.show()

# Threading for non-blocking console input
user_input_text = None
def get_console_input():
    global user_input_text
    user_input_text = input("\n[COMMANDER] Enter task (e.g., 'Find the red box') > ")

input_thread = None
rw = 0.195/2  
rb = 0.318/2  

print("\n[SYSTEM] PHASE 1: EXPLORATION STARTED.")

try:
    while plt.fignum_exists(fig.number):
        p3dx_pos = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_ori = sim.getObjectOrientation(p3dx, sim.handle_world)
        
        if last_snap_pos is None:
            last_snap_pos = p3dx_pos
            last_snap_ori = p3dx_ori[2]
            
        p3dx_mat = sim.getObjectMatrix(p3dx, sim.handle_world)
        
        T_body_world = np.array([
            [p3dx_mat[0], p3dx_mat[1], p3dx_mat[2], p3dx_mat[3]],
            [p3dx_mat[4], p3dx_mat[5], p3dx_mat[6], p3dx_mat[7]],
            [p3dx_mat[8], p3dx_mat[9], p3dx_mat[10], p3dx_mat[11]],
            [0, 0, 0, 1]
        ])

        # --- UPDATE OCCUPANCY MAP ---
        for sensor in sensor_handles:
            res, dist, point, obj, n = sim.readProximitySensor(sensor)
            hit_obstacle = True if (res and dist > 0.1) else False
            if not hit_obstacle: dist = 0.25 
            
            s_mat = sim.getObjectMatrix(sensor, p3dx)
            T_sensor_body = np.array([
                [s_mat[0], s_mat[1], s_mat[2], s_mat[3]],
                [s_mat[4], s_mat[5], s_mat[6], s_mat[7]],
                [s_mat[8], s_mat[9], s_mat[10], s_mat[11]],
                [0, 0, 0, 1]
            ])
            sensor_origin = np.matmul(T_body_world, np.matmul(T_sensor_body, np.array([[0], [0], [0], [1]])))
            sensor_hit = np.matmul(T_body_world, np.matmul(T_sensor_body, np.array([[0], [0], [dist], [1]])))
            
            my_map.update_map(sensor_origin[0][0], sensor_origin[1][0], sensor_hit[0][0], sensor_hit[1][0], hit_obstacle)

        # --- STATE MACHINE LOOP ---
        target_v, target_w = 0.0, 0.0
        current_grid_idx = my_map.world_to_grid(p3dx_pos[0], p3dx_pos[1])

        # == PHASE 1: EXPLORATION ==
        if state == "EXPLORE_PLANNING":
            grid_path = None
            binary_map = my_map.get_binary_map()
            
            # --- TIER 1: Macro Sweep ---
            active_target_idx = EXPLORATION_TARGET
            grid_path = planner.plan(binary_map, current_grid_idx, active_target_idx)
            
            # --- TIER 2: Deep Search Tool ---
            # Activates when the dummy point becomes unreachable (room is walled off)
            if not grid_path:
                print("[SLAM] Primary sweep complete. Initiating Deep Search on frontiers...")
                frontiers = get_frontier_cells(my_map.grid)
                if len(frontiers) > 0:
                    for _ in range(15):
                        active_target_idx = tuple(frontiers[random.randint(0, len(frontiers)-1)])
                        grid_path = planner.plan(binary_map, current_grid_idx, active_target_idx)
                        if grid_path: break
            
            # --- TIER 3: Endless Wander (Map is fully sealed and explored) ---
            if not grid_path:
                print("[SLAM] Room fully mapped. Wandering known space...")
                for _ in range(15):
                    free_cells = np.argwhere(my_map.grid < -0.2)
                    if len(free_cells) > 10:
                        robot_loc = np.array(current_grid_idx)
                        distances = np.linalg.norm(free_cells - robot_loc, axis=1)
                        far_cells = free_cells[distances > 15] 
                        
                        if len(far_cells) > 0:
                            active_target_idx = tuple(far_cells[random.randint(0, len(far_cells)-1)])
                        else:
                            active_target_idx = tuple(free_cells[random.randint(0, len(free_cells)-1)])
                    else:
                        active_target_idx = current_grid_idx
                    
                    grid_path = planner.plan(binary_map, current_grid_idx, active_target_idx)
                    if grid_path: break
            
            if grid_path:
                world_path = [grid_to_world(gx, gy, my_map) for gx, gy in grid_path]
                tracker.set_path(world_path)
                state = "EXPLORE_TRACKING"
            else:
                print("[SLAM] Explored all reachable bounds. No new paths found. Entering Idle Mode.")
                active_search_target = None
                state = "EXECUTION_IDLE"
                
        elif state == "EXPLORE_TRACKING":
            binary_map = my_map.get_binary_map()
            costmap = planner.inflate_map(binary_map)
            path_blocked = False
            for wx, wy in tracker.get_remaining_path()[::3]:
                dist_from_robot = math.hypot(wx - p3dx_pos[0], wy - p3dx_pos[1])
                if dist_from_robot < 0.25:
                    continue
                gx, gy = my_map.world_to_grid(wx, wy)
                if costmap[gx, gy] >= 90.0: 
                    path_blocked = True
                    break
            
            current_time = time.time()
            if current_time - last_pos_time > 1.5:
                dist_moved = math.hypot(p3dx_pos[0] - last_pos[0], p3dx_pos[1] - last_pos[1])
                if dist_moved < 0.03 and (abs(tracker.base_speed) > 0.0):
                    print("\n[CRITICAL] Physical Collision! Robot is stuck. Forcing Escape...")
                    path_blocked = True 
                    sim.setJointTargetVelocity(p3dx_lw, -1.0)
                    sim.setJointTargetVelocity(p3dx_rw, -1.0)
                    client.step()
                    time.sleep(0.5) 
                last_pos = (p3dx_pos[0], p3dx_pos[1])
                last_pos_time = current_time

            if path_blocked:
                state = "EXPLORE_PLANNING"
            else:
                dist_since_snap = math.hypot(p3dx_pos[0] - last_snap_pos[0], p3dx_pos[1] - last_snap_pos[1])
                angle_since_snap = abs(math.atan2(math.sin(p3dx_ori[2] - last_snap_ori), math.cos(p3dx_ori[2] - last_snap_ori)))
                
                if dist_since_snap >= app_config.get("snapshot_distance_m", 2.0) or math.degrees(angle_since_snap) >= app_config.get("snapshot_angle_deg", 45.0):
                    print(f"\n[SLAM] Snapshot rule triggered. Sync-blocking sim to scan...")
                    img = capture_image(sim, vision_sensor)
                    desc = agent.analyze_exploration_snapshot(img, current_grid_idx, p3dx_ori[2])
                    print(f"[MEMORY LOGGED] {desc}")
                    
                    snapshot_markers.append(current_grid_idx)
                    last_snap_pos = p3dx_pos
                    last_snap_ori = p3dx_ori[2]
                    
                    if active_search_target:
                        print(f"[SLAM] Verifying if '{active_search_target}' is in this new view...")
                        if agent.verify_target_presence(img, active_search_target):
                            print(f"[SUCCESS] Target '{active_search_target}' autonomously found during search!")
                            active_search_target = None
                            sim.setJointTargetVelocity(p3dx_lw, 0)
                            sim.setJointTargetVelocity(p3dx_rw, 0)
                            verification_markers.append((current_grid_idx[0], current_grid_idx[1], "success"))
                            state = "EXECUTION_IDLE"

                if state == "EXPLORE_TRACKING": 
                    target_v, target_w, done = tracker.compute_velocities(p3dx_pos[0], p3dx_pos[1], p3dx_ori[2])
                    if done:
                        print("\n[SLAM] Reached exploration waypoint. Picking next area...")
                        state = "EXPLORE_PLANNING"

        # == PHASE 2: EXECUTION ==
        elif state == "EXECUTION_IDLE":
            sim.setJointTargetVelocity(p3dx_lw, 0)
            sim.setJointTargetVelocity(p3dx_rw, 0)
            
            if input_thread is None or not input_thread.is_alive():
                if user_input_text is not None:
                    current_target_name = user_input_text
                    user_input_text = None
                    state = "EXECUTION_THINKING"
                else:
                    input_thread = threading.Thread(target=get_console_input)
                    input_thread.start()

        elif state == "EXECUTION_THINKING":
            print("\n[LLM] Consulting spatial memory...")
            result = agent.get_execution_target(current_target_name)
            
            print(f"[LLM Reasoning] {result.get('reasoning', '')}")
            
            if result.get("target_object"):
                current_target_name = result["target_object"]
            
            if result.get("status") == "FOUND_IN_MEMORY" and result.get("coords"):
                raw_target_idx = tuple(result["coords"])
                active_target_yaw = result.get("yaw")
                
                safe_target = get_nearest_safe_cell(my_map.get_binary_map(), planner, raw_target_idx)
                
                if safe_target:
                    active_target_idx = safe_target
                    print(f"[LLM] Dispatching to Safe Parking Spot at {active_target_idx} (Yaw: {active_target_yaw})")
                    
                    grid_path = planner.plan(my_map.get_binary_map(), current_grid_idx, active_target_idx)
                    if grid_path:
                        world_path = [grid_to_world(gx, gy, my_map) for gx, gy in grid_path]
                        tracker.set_path(world_path)
                        state = "EXECUTION_TRACKING"
                    else:
                        print("[A* ERROR] Even the safe parking spot is unreachable! Discarding memory.")
                        agent.remove_invalid_memory(list(raw_target_idx))
                        state = "EXECUTION_IDLE"
                else:
                    print("[A* ERROR] No safe parking spot exists near the target! Discarding memory.")
                    agent.remove_invalid_memory(list(raw_target_idx))
                    state = "EXECUTION_IDLE"
                    
            elif result.get("status") == "SEARCH_REQUESTED":
                print(f"[LLM] Target '{current_target_name}' not in memory. Initiating Autonomous SLAM search...")
                active_search_target = current_target_name
                state = "EXPLORE_PLANNING"
                
            else:
                print(f"[AI REPLY] {result.get('reasoning', 'I am awaiting navigation commands.')}")
                state = "EXECUTION_IDLE" 
                
        elif state == "EXECUTION_TRACKING":
            binary_map = my_map.get_binary_map()
            costmap = planner.inflate_map(binary_map)
            path_blocked = False
            
            for wx, wy in tracker.get_remaining_path()[::3]:
                dist_from_robot = math.hypot(wx - p3dx_pos[0], wy - p3dx_pos[1])
                if dist_from_robot < 0.25:
                    continue
                gx, gy = my_map.world_to_grid(wx, wy)
                if costmap[gx, gy] >= 90.0: 
                    path_blocked = True
                    break
                    
            current_time = time.time()
            if current_time - last_pos_time > 1.5:
                dist_moved = math.hypot(p3dx_pos[0] - last_pos[0], p3dx_pos[1] - last_pos[1])
                if dist_moved < 0.03 and (abs(tracker.base_speed) > 0.0):
                    print("\n[CRITICAL] Physical Collision! Robot is stuck. Forcing Escape...")
                    path_blocked = True 
                    sim.setJointTargetVelocity(p3dx_lw, -1.0)
                    sim.setJointTargetVelocity(p3dx_rw, -1.0)
                    client.step()
                    time.sleep(0.5) 
                last_pos = (p3dx_pos[0], p3dx_pos[1])
                last_pos_time = current_time
            
            if path_blocked:
                print("[A*] Obstacle detected on path. Rerouting to target...")
                safe_target = get_nearest_safe_cell(my_map.get_binary_map(), planner, active_target_idx)
                
                if safe_target:
                    active_target_idx = safe_target
                    grid_path = planner.plan(my_map.get_binary_map(), current_grid_idx, active_target_idx)
                    
                    if grid_path:
                        world_path = [grid_to_world(gx, gy, my_map) for gx, gy in grid_path]
                        tracker.set_path(world_path)
                    else:
                        print("[A*] Target totally blocked off. Giving up.")
                        state = "EXECUTION_IDLE"
                else:
                    print("[A*] Safe parking spot blocked by dynamic obstacle. Giving up.")
                    state = "EXECUTION_IDLE"
            else:
                target_v, target_w, done = tracker.compute_velocities(p3dx_pos[0], p3dx_pos[1], p3dx_ori[2])
                if done:
                    if active_target_yaw is not None:
                        print(f"[STATE] Destination reached. Orienting to {math.degrees(active_target_yaw):.1f} degrees...")
                        state = "EXECUTION_ORIENTING"
                    else:
                        print("[STATE] Destination reached. Initiating Verification...")
                        state = "EXECUTION_VERIFYING"
                        
        elif state == "EXECUTION_ORIENTING":
            angle_error = active_target_yaw - p3dx_ori[2]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            if abs(angle_error) < 0.15: 
                sim.setJointTargetVelocity(p3dx_lw, 0)
                sim.setJointTargetVelocity(p3dx_rw, 0)
                print("[STATE] Target orientation aligned. Initiating Verification...")
                state = "EXECUTION_VERIFYING"
            else:
                target_v = 0.0
                target_w = 0.8 * angle_error 
                target_w = max(-1.0, min(1.0, target_w))
                
        elif state == "EXECUTION_VERIFYING":
            sim.setJointTargetVelocity(p3dx_lw, 0)
            sim.setJointTargetVelocity(p3dx_rw, 0)
            client.step()
            time.sleep(0.5) 
            
            img = capture_image(sim, vision_sensor)
            is_valid = agent.verify_target_presence(img, current_target_name)
            
            if is_valid:
                print(f"[SUCCESS] '{current_target_name}' visually confirmed at location.")
                verification_markers.append((current_grid_idx[0], current_grid_idx[1], "success"))
                state = "EXECUTION_IDLE"
            else:
                print(f"[FAILURE] '{current_target_name}' is missing! Invalidating memory...")
                agent.remove_invalid_memory(list(active_target_idx))
                verification_markers.append((current_grid_idx[0], current_grid_idx[1], "fail"))
                print("[SYSTEM] Reverting to Exploration to find relocated target...")
                active_search_target = current_target_name
                state = "EXPLORE_PLANNING"

        # --- APPLY WHEEL VELOCITIES ---
        vl = (target_v - (rb * target_w)) / rw
        vr = (target_v + (rb * target_w)) / rw
        sim.setJointTargetVelocity(p3dx_lw, vl)
        sim.setJointTargetVelocity(p3dx_rw, vr)
        
        client.step()

        # --- UPDATE GUI ---
        loop_counter += 1
        if loop_counter % 10 == 0:
            im.set_data(my_map.grid.T)
            rx, ry = my_map.world_to_grid(p3dx_pos[0], p3dx_pos[1])
            robot_dot.set_data([rx], [ry])
            
            if snapshot_markers:
                mx, my = zip(*snapshot_markers)
                memory_stars.set_data(mx, my)
                
            if verification_markers:
                vx, vy = zip(*[(m[0], m[1]) for m in verification_markers if m[2] == "success"])
                verify_dots.set_data(vx, vy)
                
            if world_path and "TRACKING" in state:
                px, py = zip(*[my_map.world_to_grid(x, y) for x, y in world_path])
                path_line.set_data(px, py)
            else:
                path_line.set_data([], [])

            fig.canvas.flush_events()
            
        plt.pause(0.01)

except Exception as e:
    print(f"\nSystem halted: {e}")

finally:
    sim.setJointTargetVelocity(p3dx_lw, 0)
    sim.setJointTargetVelocity(p3dx_rw, 0)
    sim.stopSimulation()
    print("Simulation Terminated.")