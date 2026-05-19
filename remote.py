import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from mapper import OccupancyGridMap

# Unbind default keys so it doesn't interfere with WASD controls
if 's' in mpl.rcParams['keymap.save']:
    mpl.rcParams['keymap.save'].remove('s')
if 'q' in mpl.rcParams['keymap.quit']:
    mpl.rcParams['keymap.quit'].remove('q')

client = RemoteAPIClient()
sim = client.require('sim')

client.setStepping(True) 
sim.startSimulation()
print("Simulation Started. Generating map live...")

p3dx = sim.getObject("/PioneerP3DX")
p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")

sensor_handles = [
    sim.getObject("/PioneerP3DX/ultrasonicSensor[0]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[1]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[2]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[3]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[4]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[5]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[6]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[7]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[8]")
]

# Initialize map 
my_map = OccupancyGridMap(width_m=30.0, height_m=30.0, resolution=0.1)
my_map.l_free = -0.5

keys = {'w': False, 's': False, 'a': False, 'd': False, 'q': False}

def on_press(event):
    if event.key in keys: keys[event.key] = True

def on_release(event):
    if event.key in keys: keys[event.key] = False

plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.mpl_connect('key_press_event', on_press)
fig.canvas.mpl_connect('key_release_event', on_release)

im = ax.imshow(my_map.grid.T, cmap='Greys', origin='lower', vmin=-3, vmax=3)
ax.set_title("Live Map - Click here, use WASD to drive, Q to quit")
plt.show()

max_sensor_range = 1.0 
loop_counter = 0

# Variables for smooth acceleration
current_v = 0.0
current_w = 0.0
alpha = 0.15 # Smoothing factor (lower = smoother)

try:
    while not keys['q']:
        
        # Get Robot Position & Transform Matrix
        p3dx_pos = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_mat = sim.getObjectMatrix(p3dx, sim.handle_world)
        T_body_world = np.array([
            [p3dx_mat[0], p3dx_mat[1], p3dx_mat[2], p3dx_mat[3]],
            [p3dx_mat[4], p3dx_mat[5], p3dx_mat[6], p3dx_mat[7]],
            [p3dx_mat[8], p3dx_mat[9], p3dx_mat[10], p3dx_mat[11]],
            [0, 0, 0, 1]
        ])

        # Read all sensors
        for sensor in sensor_handles:
            res, dist, point, obj, n = sim.readProximitySensor(sensor)
            
            # SCENARIO A: Sensor hit a wall. Trust the data.
            if res and dist > 0.1:
                hit_obstacle = True
            
            # SCENARIO B: Sensor hit nothing.
            else:
                hit_obstacle = False
                dist = 0.5 

            # Get sensor transform relative to the robot body
            s_mat = sim.getObjectMatrix(sensor, p3dx)
            T_sensor_body = np.array([
                [s_mat[0], s_mat[1], s_mat[2], s_mat[3]],
                [s_mat[4], s_mat[5], s_mat[6], s_mat[7]],
                [s_mat[8], s_mat[9], s_mat[10], s_mat[11]],
                [0, 0, 0, 1]
            ])
            
            # Calculate the starting position of the sensor in the world
            sensor_origin_local = np.array([[0], [0], [0], [1]])
            sensor_origin_body = np.matmul(T_sensor_body, sensor_origin_local)
            sensor_origin_world = np.matmul(T_body_world, sensor_origin_body)
            
            # Calculate the hit position in the world
            sensor_point_local = np.array([[0], [0], [dist], [1]]) 
            sensor_reading_body = np.matmul(T_sensor_body, sensor_point_local)
            sensor_reading_world = np.matmul(T_body_world, sensor_reading_body)
            
            # Update Map using the coordinates as the starting point
            my_map.update_map(
                robot_x=sensor_origin_world[0][0], 
                robot_y=sensor_origin_world[1][0], 
                sensor_hit_x=sensor_reading_world[0][0], 
                sensor_hit_y=sensor_reading_world[1][0], 
                hit_obstacle=hit_obstacle
            )

        # Calculate target velocities
        target_v = 0.0
        target_w = 0.0
        base_speed = 10.0
        turn_speed = 5.0

        if keys['w']: target_v += base_speed
        if keys['s']: target_v -= base_speed
        if keys['a']: target_w += turn_speed 
        if keys['d']: target_w -= turn_speed 

        # Smooth velocity interpolation to eliminate jitter
        current_v = current_v * (1 - alpha) + target_v * alpha
        current_w = current_w * (1 - alpha) + target_w * alpha

        vl = current_v - current_w
        vr = current_v + current_w

        sim.setJointTargetVelocity(p3dx_lw, vl)
        sim.setJointTargetVelocity(p3dx_rw, vr)

        # Trigger the next simulator step (Synchronous mode)
        client.step()

        # Update visual plot every 10 iterations
        loop_counter += 1
        if loop_counter % 10 == 0:
            im.set_data(my_map.grid.T) 
            fig.canvas.flush_events()
            
        plt.pause(0.01) 

except Exception as e:
    print(f"\nStopped due to: {e}")

finally:
    sim.setJointTargetVelocity(p3dx_lw, 0)
    sim.setJointTargetVelocity(p3dx_rw, 0)
    sim.stopSimulation()
    
    # Export clean binary map
    binary_map = (my_map.grid > 0).astype(np.uint8)
    plt.imsave('final_map.png', binary_map.T, cmap='Greys', origin='lower')
    print("\nSimulation Stopped. Final map saved as 'final_map.png'.")