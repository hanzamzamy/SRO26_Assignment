import math
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

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
start_position    = [0.6, -0.125, 0.13879]
start_orientation = [0.0, 0.0, 0.0]

sim.setObjectPosition(p3dx, sim.handle_world, start_position)
sim.setObjectOrientation(p3dx, sim.handle_world, start_orientation)

# Initialize camera (position and orientation)
camera_position    = [0.0, 0.0, 8.0]
camera_orientation = [math.pi, 0.0, math.pi]

sim.setObjectPosition(camera, sim.handle_world, camera_position)
sim.setObjectOrientation(camera, sim.handle_world, camera_orientation)

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
x_odom_data = []
y_odom_data = []
x_odom_abs_data = []
y_odom_abs_data = []
x_true_data = []
y_true_data = []

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
        
            # Getting actual joint velocity for the plots
            wr_vel = sim.getJointVelocity(right_wheel)
            wl_vel = sim.getJointVelocity(left_wheel)

            # Calculation based on the kinematic model of a differential drive robot
            vx = (rw / 2.0) * (wr_vel + wl_vel)
            wx = (rw / L) * (wr_vel - wl_vel)

            # ------------------------------------------
            # Absolute angle odometry calculation (using world frame orientation)
            # ------------------------------------------

            # Get the current orientation of the robot in the world frame
            euler_angle = sim.getObjectOrientation(p3dx, sim.handle_world)

            x_dot_abs = vx * math.cos(euler_angle[2])
            y_dot_abs = vx * math.sin(euler_angle[2])

            x_dot_abs_integrated += x_dot_abs * dt
            y_dot_abs_integrated += y_dot_abs * dt

            # sim.addLog(1, f"x_dot_abs: {x_dot_abs:.2f} m/s | y_dot_abs: {y_dot_abs:.2f} m/s | x_abs: {x_dot_abs_integrated:.2f} m | y_abs: {y_dot_abs_integrated:.2f} m")

            # ------------------------------------------

            # ------------------------------------------
            # Relative angle odometry calculation (integrating angular velocity to get orientation)
            # ------------------------------------------

            wx_integrated += wx * dt

            x_dot = vx * math.cos(wx_integrated)
            y_dot = vx * math.sin(wx_integrated)

            x_dot_integrated += x_dot * dt
            y_dot_integrated += y_dot * dt

            # sim.addLog(1, f"x_dot: {x_dot:.2f} m/s | y_dot: {y_dot:.2f} m/s | x: {x_dot_integrated:.2f} m | y: {y_dot_integrated:.2f} m")

            # ------------------------------------------

            # ------------------------------------------
            # Ground truth data for comparison
            # ------------------------------------------

            true_position = sim.getObjectPosition(p3dx, sim.handle_world)

            # ------------------------------------------

            # Append data to lists
            x_odom_data.append(x_dot_integrated)
            y_odom_data.append(y_dot_integrated)
            x_odom_abs_data.append(x_dot_abs_integrated)
            y_odom_abs_data.append(y_dot_abs_integrated)
            x_true_data.append(true_position[0])
            y_true_data.append(true_position[1])
            
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
    plt.figure(figsize=(10, 8))
    
    # Spatial plot of P3DX odometry position
    plt.plot(x_odom_data, y_odom_data, label='Relative Angle Odometry Path', color='green')
    plt.plot(x_odom_abs_data, y_odom_abs_data, label='Absolute Angle Odometry Path', color='blue')
    plt.plot(x_true_data, y_true_data, label='Ground Truth Path', color='black', linestyle='--')

    # Plot start and end points
    plt.scatter(x_odom_data[0], y_odom_data[0], color='green', marker='o', label='Rel. Start')
    plt.scatter(x_odom_data[-1], y_odom_data[-1], color='green', marker='x', label='Rel. End')
    plt.scatter(x_odom_abs_data[0], y_odom_abs_data[0], color='blue', marker='o', label='Abs. Start')
    plt.scatter(x_odom_abs_data[-1], y_odom_abs_data[-1], color='blue', marker='x', label='Abs. End')
    plt.scatter(x_true_data[0], y_true_data[0], color='black', marker='o', label='True Start')
    plt.scatter(x_true_data[-1], y_true_data[-1], color='black', marker='x', label='True End')

    # Draw walls for reference
    wall_xp_pos = sim.getObjectPosition(wall_xp, sim.handle_world)
    wall_xm_pos = sim.getObjectPosition(wall_xm, sim.handle_world)
    wall_yp_pos = sim.getObjectPosition(wall_yp, sim.handle_world)
    wall_ym_pos = sim.getObjectPosition(wall_ym, sim.handle_world)
    plt.plot([wall_xp_pos[0], wall_xp_pos[0]], [-3, 3], color='red', linestyle=':')
    plt.plot([wall_xm_pos[0], wall_xm_pos[0]], [-3, 3], color='red', linestyle=':')
    plt.plot([-3, 3], [wall_yp_pos[1], wall_yp_pos[1]], color='red', linestyle=':')
    plt.plot([-3, 3], [wall_ym_pos[1], wall_ym_pos[1]], color='red', linestyle=':')

    # Plot settings
    plt.title('Spatial Plot of P3DX Odometry Position')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.xlim(-3, 3)
    plt.ylim(-3, 3)
    plt.grid(True)
    plt.legend()
    
    # plt.tight_layout()
    plt.show()

    # ------------------------------------------

# ==========================================
