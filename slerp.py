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
cb3 = sim.getObject("/Cuboid[2]")

try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    t_max = 3.0  # Total time for interpolation (in seconds)
    while (time.time() - start_time) < t_max:

        def quaternion_slerp(q1, q2, t):
            """
            Spherical Linear Interpolation antara dua quaternion q1 dan q2.
            Parameter t berada di range [0.0, 1.0].
            """
            # Normalisasi quaternion
            q1 = q1 / np.linalg.norm(q1)
            q2 = q2 / np.linalg.norm(q2)

            # Hitung dot product
            dot = np.dot(q1, q2)

            # Jika dot product negatif, gunakan -q2 untuk interpolasi yang lebih pendek
            if dot < 0.0:
                q2 = -q2
                dot = -dot

            # Threshold untuk menghindari masalah numerik
            DOT_THRESHOLD = 0.9995
            if dot > DOT_THRESHOLD:
                # Jika quaternions sangat dekat, gunakan linear interpolation
                result = q1 + t * (q2 - q1)
                return result / np.linalg.norm(result)

            # Hitung sudut antara quaternions dan lakukan interpolasi
            theta_0 = math.acos(dot)  # Sudut awal antara q1 dan q2
            theta = theta_0 * t       # Sudut interpolasi saat ini

            sin_theta_0 = math.sin(theta_0)
            sin_theta = math.sin(theta)

            s1 = math.cos(theta) - dot * sin_theta / sin_theta_0
            s2 = sin_theta / sin_theta_0

            return (s1 * q1) + (s2 * q2)

        # Convert Euler angles to quaternions (CoppeliaSim X-Y-Z Intrinsic Convention)
        def euler_to_quat_coppelia(euler):
            alpha, beta, gamma = euler
            
            # Setengah sudut
            cx = math.cos(alpha * 0.5)
            sx = math.sin(alpha * 0.5)
            cy = math.cos(beta * 0.5)
            sy = math.sin(beta * 0.5)
            cz = math.cos(gamma * 0.5)
            sz = math.sin(gamma * 0.5)

            # Perhitungan Quaternion X-Y-Z
            w = cx * cy * cz - sx * sy * sz
            x = sx * cy * cz + cx * sy * sz
            y = cx * sy * cz - sx * cy * sz
            z = cx * cy * sz + sx * sy * cz
            
            return np.array([x, y, z, w])
        

        # get euler angles (alpha, beta, gamma) of cb1 and cb2
        euler_cube1 = sim.getObjectOrientation(cb1, sim.handle_world)  # returns (alpha, beta, gamma) in radians
        euler_cube2 = sim.getObjectOrientation(cb2, sim.handle_world)

        # Calculate quaternions from Euler angles
        quat_cube1_from_euler = euler_to_quat_coppelia(euler_cube1)
        quat_cube2_from_euler = euler_to_quat_coppelia(euler_cube2)

        q_start = quat_cube1_from_euler # Identitas
        q_end = quat_cube2_from_euler # Rotasi 90 derajat di sumbu Y
        t_now = time.time() - start_time
        t = t_now / t_max  # Normalisasi waktu ke dalam range [0, 1]

        q_mid = quaternion_slerp(q_start, q_end, t)

        # Set cb3 orientation using the interpolated quaternion
        sim.setObjectQuaternion(cb3, sim.handle_world, q_mid.tolist())

    t_post = 5.0

    while (time.time() - start_time) < t_post + t_max:
        time.sleep(0.1)  # Tunggu sebentar sebelum mengakhiri simulasi

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")