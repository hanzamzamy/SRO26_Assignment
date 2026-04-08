import math
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np

# ==========================================
# Setup & Initialization
# ==========================================
client = RemoteAPIClient()
sim = client.require('sim')

# Get robot handles
p3dx = sim.getObject('/PioneerP3DX')
right_wheel = sim.getObject('/PioneerP3DX/rightMotor')
left_wheel = sim.getObject('/PioneerP3DX/leftMotor')

# Get wall handles
wall_xp = sim.getObject('/wall_xp')
wall_xm = sim.getObject('/wall_xm')
wall_yp = sim.getObject('/wall_yp')
wall_ym = sim.getObject('/wall_ym')

# Get camera handle
camera = sim.getObject('/DefaultCamera')

# Get sphere object
sphere = sim.getObject('/Sphere')

# ------------------------------------------
# Environment Initialization
# ------------------------------------------

# Initialize walls (position and orientation)
wall_height    = 0.5 
wall_thickness = 0.1
wall_length    = 5.0 # Floor size

sim.setObjectPosition(wall_xp, sim.handle_world, [((wall_length / 2) + wall_thickness / 2), 0.0, wall_height / 2])
sim.setObjectPosition(wall_xm, sim.handle_world, [-((wall_length / 2) + wall_thickness / 2), 0.0, wall_height / 2])
sim.setObjectPosition(wall_yp, sim.handle_world, [0.0, ((wall_length / 2) + wall_thickness / 2), wall_height / 2])
sim.setObjectPosition(wall_ym, sim.handle_world, [0.0, -((wall_length / 2) + wall_thickness / 2), wall_height / 2])

sim.setObjectOrientation(wall_yp, sim.handle_world, [0.0, 0.0, 0.0])
sim.setObjectOrientation(wall_ym, sim.handle_world, [0.0, 0.0, 0.0])
sim.setObjectOrientation(wall_xp, sim.handle_world, [0.0, 0.0, math.pi / 2])
sim.setObjectOrientation(wall_xm, sim.handle_world, [0.0, 0.0, math.pi / 2])

# Initialize robot (position and orientation)
start_position    = [-2.0, -0.5, 0.13879]
start_orientation = [0.0, 0.0, 0.0]

sim.setObjectPosition(p3dx, sim.handle_world, start_position)
sim.setObjectOrientation(p3dx, sim.handle_world, start_orientation)

# Initialize camera (position and orientation)
camera_position    = [0.0, 0.0, 8.0]
camera_orientation = [math.pi, 0.0, math.pi]

sim.setObjectPosition(camera, sim.handle_world, camera_position)
sim.setObjectOrientation(camera, sim.handle_world, camera_orientation)

# Initialize sphere (position and orientation)
sphere_position    = [1.0, 0.8, 0.5]
sphere_orientation = [0.0, 0.0, 0.0]

sim.setObjectPosition(sphere, sim.handle_world, sphere_position)
sim.setObjectOrientation(sphere, sim.handle_world, sphere_orientation)

# ------------------------------------------

# Start Simulation
sim.startSimulation()
print("Simulation Started")

# Parameters
rw = 0.195 / 2.0 # Wheel radius (half of the diameter)
# rb = 0.381 / 2 # This makes the angular velocity twice as fast, which is incorrect.
L = 0.381       # Distance between the wheels (track width)

run_time = 90.0  # Run the simulation

# Odometry variables
x_dot_integrated = 0.0
y_dot_integrated = 0.0
x_dot_abs_integrated = 0.0
y_dot_abs_integrated = 0.0
wx_integrated = 0.0
dt = 0.01

# Lists to store data for plotting
# x_odom_data = []
# y_odom_data = []
# x_odom_abs_data = []
# y_odom_abs_data = []
# x_true_data = []
# y_true_data = []

# ==========================================

# ==========================================
# Simulation Loop
# ==========================================
try:
    # start_time = time.time()
    # last_time = start_time

    # Use simulation time instead of wall clock
    start_time = sim.getSimulationTime()
    last_time = start_time
    
    while True:
        # current_time = time.time()
        current_time = sim.getSimulationTime()
        elapsed = current_time - start_time
        
        if elapsed >= run_time:
            break
            
        print(f"Running... {elapsed:.1f}s", end="\r")

        dt = current_time - last_time
        
        # Do stuffes only if simulation time has advanced
        if dt > 0:
            last_time = current_time

            p3dx_pos = sim.getObjectPosition(p3dx, sim.handle_world)
            p3dx_orn = sim.getObjectOrientation(p3dx, sim.handle_world)

            sphere_pos = sim.getObjectPosition(sphere, sim.handle_world)

            T_world_p3dx_2D = np.array([[math.cos(p3dx_orn[2]), -math.sin(p3dx_orn[2]), p3dx_pos[0]],
                                        [math.sin(p3dx_orn[2]),  math.cos(p3dx_orn[2]), p3dx_pos[1]],
                                        [0,                     0,                      1]])
            
            sphere_pos_p3dx_frame = np.linalg.inv(T_world_p3dx_2D) @ np.array([[sphere_pos[0]], [sphere_pos[1]], [1]])

            sim.addLog(1, f"Sphere w.r.t. P3DX: ({sphere_pos_p3dx_frame[0, 0]:.2f}, {sphere_pos_p3dx_frame[1, 0]:.2f})")

            Kp = 2.5
            vx = Kp * (math.sqrt(sphere_pos_p3dx_frame[0, 0]**2 + sphere_pos_p3dx_frame[1, 0]**2) - 0.8)
            wx = Kp * math.atan2(sphere_pos_p3dx_frame[1, 0], sphere_pos_p3dx_frame[0, 0])

            wr = (vx + wx/2)
            wl = (vx - wx/2)

            sim.setJointTargetVelocity(right_wheel, wr)
            sim.setJointTargetVelocity(left_wheel, wl)

            # Append data to lists
            # x_odom_data.append(x_dot_integrated)
            # y_odom_data.append(y_dot_integrated)
            # x_odom_abs_data.append(x_dot_abs_integrated)
            # y_odom_abs_data.append(y_dot_abs_integrated)
            # x_true_data.append(true_position[0])
            # y_true_data.append(true_position[1])
            
        # time.sleep(0.05)

# ==========================================

# ==========================================
# Finalize & Cleanup
# ==========================================
finally:
    # Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")
    
    # ------------------------------------------
    # Plotting the results
    # ------------------------------------------
    # plt.figure(figsize=(10, 8))
    
    # # Spatial plot of P3DX odometry position
    # plt.plot(x_odom_data, y_odom_data, label='Relative Angle Odometry Path', color='green')
    # plt.plot(x_odom_abs_data, y_odom_abs_data, label='Absolute Angle Odometry Path', color='blue')
    # plt.plot(x_true_data, y_true_data, label='Ground Truth Path', color='black', linestyle='--')

    # # Plot start and end points
    # plt.scatter(x_odom_data[0], y_odom_data[0], color='green', marker='o', label='Rel. Start')
    # plt.scatter(x_odom_data[-1], y_odom_data[-1], color='green', marker='x', label='Rel. End')
    # plt.scatter(x_odom_abs_data[0], y_odom_abs_data[0], color='blue', marker='o', label='Abs. Start')
    # plt.scatter(x_odom_abs_data[-1], y_odom_abs_data[-1], color='blue', marker='x', label='Abs. End')
    # plt.scatter(x_true_data[0], y_true_data[0], color='black', marker='o', label='True Start')
    # plt.scatter(x_true_data[-1], y_true_data[-1], color='black', marker='x', label='True End')

    # # Draw walls for reference
    # wall_xp_pos = sim.getObjectPosition(wall_xp, sim.handle_world)
    # wall_xm_pos = sim.getObjectPosition(wall_xm, sim.handle_world)
    # wall_yp_pos = sim.getObjectPosition(wall_yp, sim.handle_world)
    # wall_ym_pos = sim.getObjectPosition(wall_ym, sim.handle_world)
    # plt.plot([wall_xp_pos[0], wall_xp_pos[0]], [-3, 3], color='red', linestyle=':')
    # plt.plot([wall_xm_pos[0], wall_xm_pos[0]], [-3, 3], color='red', linestyle=':')
    # plt.plot([-3, 3], [wall_yp_pos[1], wall_yp_pos[1]], color='red', linestyle=':')
    # plt.plot([-3, 3], [wall_ym_pos[1], wall_ym_pos[1]], color='red', linestyle=':')

    # # Plot settings
    # plt.title('Spatial Plot of P3DX Odometry Position')
    # plt.xlabel('x (m)')
    # plt.ylabel('y (m)')
    # plt.xlim(-3, 3)
    # plt.ylim(-3, 3)
    # plt.grid(True)
    # plt.legend()
    
    # # plt.tight_layout()
    # plt.show()

    # ------------------------------------------

# ==========================================
