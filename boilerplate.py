# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")


# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
cb1 = sim.getObject("/Cuboid[0]")
cb2 = sim.getObject("/Cuboid[1]")

pt1 = sim.getObject("/Sphere[0]")
pt2 = sim.getObject("/Sphere[1]")
pt3 = sim.getObject("/Sphere[2]")

try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    while (time.time() - start_time) < 60*10:

        # Get Euler angles of Cuboid[0] and Cuboid[1]
        euler_cube1 = sim.getObjectOrientation(cb1, sim.handle_world)
        euler_cube2 = sim.getObjectOrientation(cb2, sim.handle_world)

        # Convert Euler angles to YPR rotation matrices
        def rotation_matrix(euler_angles):
            roll, pitch, yaw = euler_angles
            # X-axis rotation (roll)
            R_x = np.array([[1, 0, 0],
                             [0, np.cos(roll), -np.sin(roll)],
                             [0, np.sin(roll), np.cos(roll)]])

            # Y-axis rotation (pitch)
            R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                             [0, 1, 0],
                             [-np.sin(pitch), 0, np.cos(pitch)]])

            # Z-axis rotation (yaw)
            R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                             [np.sin(yaw), np.cos(yaw), 0],
                             [0, 0, 1]])

            # Combine rotations: R = R_z * R_y * R_x
            return R_z @ R_y @ R_x

        # Compute full rotation matrices
        R_cube1 = rotation_matrix(euler_cube1)
        R_cube2 = rotation_matrix(euler_cube2)

        # Print matrices
        print("Rotation matrix for Cube 1 (RPY):\n", R_cube1)
        print("Rotation matrix for Cube 2 (RPY):\n", R_cube2)

        # Compute rotation matrix of Cube 2 relative to Cube 1
        R_rel = R_cube1.T @ R_cube2  # since R^{-1} = R^T for rotation matrices

        sim.setObjectPosition(pt1, R_rel[:, 0].tolist(), cb1)
        sim.setObjectPosition(pt2, R_rel[:, 1].tolist(), cb1)
        sim.setObjectPosition(pt3, R_rel[:, 2].tolist(), cb1)
        # time.sleep(1)

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")
