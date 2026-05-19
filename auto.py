import time
import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from mapper import OccupancyGridMap
from astar import AStarPlanner
from tracker import PurePursuitTracker

def grid_to_world(x_grid, y_grid, map_obj):
    x_world = (x_grid * map_obj.resolution) - map_obj.offset_x
    y_world = (y_grid * map_obj.resolution) - map_obj.offset_y
    return x_world, y_world

client = RemoteAPIClient()
sim = client.require('sim')
client.setStepping(True) 
sim.startSimulation()
print("Autonomous Navigation Started!")

p3dx = sim.getObject("/PioneerP3DX")
p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")

sensor_handles = [sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]") for i in range(9)]


my_map = OccupancyGridMap(width_m=30.0, height_m=30.0, resolution=0.1)
my_map.l_free = -0.5
planner = AStarPlanner(resolution=0.1, lethal_radius_m=0.3, risk_radius_m=0.5)
tracker = PurePursuitTracker(lookahead_distance=0.4, base_speed=0.8, stop_tolerance=0.15)

goal_grid_idx = None

def on_click(event):
    global goal_grid_idx
    if event.xdata is not None and event.ydata is not None:
        goal_grid_idx = (int(event.xdata), int(event.ydata))
        print(f"\n[UI] New Target Clicked: {goal_grid_idx}")

plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.mpl_connect('button_press_event', on_click)

im = ax.imshow(my_map.grid.T, cmap='Greys', origin='lower', vmin=-3, vmax=3)
path_line, = ax.plot([], [], 'r-', linewidth=2, label="A* Path")
robot_dot, = ax.plot([], [], 'bo', markersize=6, label="Robot")
goal_dot, = ax.plot([], [], 'go', markersize=8, label="Goal")
ax.set_title("Live Map")
plt.legend()
plt.show()


rw = 0.195/2  
rb = 0.318/2  
loop_counter = 0

state = "WAITING" 
active_goal_idx = None
world_path = []

last_pos = (0, 0)
last_pos_time = time.time()

try:
    while plt.fignum_exists(fig.number):
        p3dx_pos = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_ori = sim.getObjectOrientation(p3dx, sim.handle_world)
        p3dx_mat = sim.getObjectMatrix(p3dx, sim.handle_world)
        
        T_body_world = np.array([
            [p3dx_mat[0], p3dx_mat[1], p3dx_mat[2], p3dx_mat[3]],
            [p3dx_mat[4], p3dx_mat[5], p3dx_mat[6], p3dx_mat[7]],
            [p3dx_mat[8], p3dx_mat[9], p3dx_mat[10], p3dx_mat[11]],
            [0, 0, 0, 1]
        ])

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
            
            sensor_origin_world = np.matmul(T_body_world, np.matmul(T_sensor_body, np.array([[0], [0], [0], [1]])))
            sensor_reading_world = np.matmul(T_body_world, np.matmul(T_sensor_body, np.array([[0], [0], [dist], [1]])))
            
            my_map.update_map(
                robot_x=sensor_origin_world[0][0], robot_y=sensor_origin_world[1][0], 
                sensor_hit_x=sensor_reading_world[0][0], sensor_hit_y=sensor_reading_world[1][0], 
                hit_obstacle=hit_obstacle
            )

        if goal_grid_idx != active_goal_idx:
            print("[STATE] User requested new target. Cancelling current route...")
            active_goal_idx = goal_grid_idx
            state = "PLANNING"

        if state == "PLANNING" and active_goal_idx is not None:
            print("[STATE] Planning...")
            sim.setJointTargetVelocity(p3dx_lw, 0)
            sim.setJointTargetVelocity(p3dx_rw, 0)
            
            start_grid_idx = my_map.world_to_grid(p3dx_pos[0], p3dx_pos[1])
            binary_map = my_map.get_binary_map()
            
            grid_path = planner.plan(binary_map, start_grid_idx, active_goal_idx)
            
            if grid_path:
                print("[STATE] Path found! Tracking...")
                world_path = [grid_to_world(gx, gy, my_map) for gx, gy in grid_path]
                tracker.set_path(world_path)
                state = "TRACKING"
            else:
                print("[STATE] Target Unreachable. Waiting for new input.")
                goal_grid_idx = None 
                active_goal_idx = None
                world_path = []
                state = "WAITING"

        target_v, target_w = 0.0, 0.0
        
        if state == "TRACKING":
            binary_map = my_map.get_binary_map()
            costmap = planner.inflate_map(binary_map) 
            
            path_blocked = False
            remaining_path = tracker.get_remaining_path() 
            
            for wx, wy in remaining_path[::3]:
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
                print("[STATE] Obstacle Ahead! Replanning...")
                state = "PLANNING"
            
            else:
                target_v, target_w, done = tracker.compute_velocities(p3dx_pos[0], p3dx_pos[1], p3dx_ori[2])
                if done:
                    print("[STATE] Target Reached!")
                    goal_grid_idx = None
                    active_goal_idx = None
                    world_path = []
                    state = "WAITING"

        vl = (target_v - (rb * target_w)) / rw
        vr = (target_v + (rb * target_w)) / rw
        sim.setJointTargetVelocity(p3dx_lw, vl)
        sim.setJointTargetVelocity(p3dx_rw, vr)
        client.step()

        loop_counter += 1
        if loop_counter % 10 == 0:
            im.set_data(my_map.grid.T)
            
            rx, ry = my_map.world_to_grid(p3dx_pos[0], p3dx_pos[1])
            robot_dot.set_data([rx], [ry])
            
            if active_goal_idx:
                goal_dot.set_data([active_goal_idx[0]], [active_goal_idx[1]])
                if len(world_path) > 0:
                    px, py = zip(*[my_map.world_to_grid(x, y) for x, y in world_path])
                    path_line.set_data(px, py)
            else:
                goal_dot.set_data([], [])
                path_line.set_data([], [])

            fig.canvas.flush_events()
        plt.pause(0.01) 

except Exception as e:
    print(f"\nStopped due to: {e}")

finally:
    sim.setJointTargetVelocity(p3dx_lw, 0)
    sim.setJointTargetVelocity(p3dx_rw, 0)
    sim.stopSimulation()
    print("Simulation Stopped.")