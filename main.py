import sys
import time
import math
import json
import argparse
import random
import threading
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# LOAD DOTENV FIRST before doing anything else
from dotenv import load_dotenv
load_dotenv()

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from mapper import OccupancyGridMap
from astar import AStarPlanner
from tracker import PurePursuitTracker
from llm_agent import GeminiNavigator
# from ollama_agent import OllamaNavigator

# --- CLI ARGUMENT PARSER (FOR GRADING HARNESS) ---
parser = argparse.ArgumentParser(description="P3DX Autonomous Semantic Agent")
parser.add_argument('--mode', type=str, default='demo', choices=['demo', 'harness'], help="Run mode")
parser.add_argument('--state', type=str, help="State JSON string (Harness mode)")
args = parser.parse_args()

# HARNESS MODE EXECUTION (1-shot API test for the grader)
if args.mode == 'harness':
    if not args.state:
        print(json.dumps({"error": "--state argument required for harness mode."}))
        sys.exit(1)
    
    agent = GeminiNavigator()
    # agent = OllamaNavigator()
    result = agent.get_execution_target(args.state)
    print(json.dumps({"action": "PRESET_OPEN_LOOP_TARGET", "payload": result}))
    sys.exit(0)

# =======================================================================
# === DEMO MODE EXECUTION (Full Closed-Loop Orchestrator) ===============
# =======================================================================

# Load configuration rules
with open("config.json", "r") as f:
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

# Auto.py Stuck Escaping Tracking Variables
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
            
            # --- TIER 1: Macro Sweep (Your Original Dummy Point) ---
            active_target_idx = EXPLORATION_TARGET
            grid_path = planner.plan(binary_map, current_grid_idx, active_target_idx)
            
            # --- TIER 2: Deep Search Tool (Frontiers) ---
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