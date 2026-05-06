# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def transformMat(alpha, beta, gamma, tx, ty, tz):
    # 1. Individual Rotation Matrices (3x3)
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
        ])
    roty = np.array([
        [ math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
        ])
    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0,0,1]
        ])
    # 2. Total Rotation Matrix (R_total, 3x3)
    rot_total = np.matmul(rotx, roty)
    rot_total = np.matmul(rot_total, rotz)
    # 3. Translation Vector (t, 3x1)
    trans_vector = np.array([
                    [tx],
                    [ty],
                    [tz]
                    ])
    # 4. Create the 3x4 Transformation Matrix
    R_t_3x4  = np.hstack((rot_total, trans_vector))
    # 5. Create the Homogeneous Row [0 0 0 1] (1x4)
    homogeneous_row = np.array([[0, 0, 0, 1]])
    # 6. Vertically stack to create the 4x4 Homogeneous Transformation Matrix
    transform_matrix_4x4 = np.vstack((R_t_3x4, homogeneous_row))
    return transform_matrix_4x4

# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")


# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
p3dx = sim.getObject("/PioneerP3DX")
s3_Handle = sim.getObject("/ultrasonicSensor[3]")

rw = 0.195/2
rb = 0.318/2
d = 0.05

dt = 0.01

z_sensor = []
t = []
x_sensor_to_body = []
y_sensor_to_body = []
x_sensor_to_world = []
y_sensor_to_world = []

try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0.0
    while (time.time() - start_time) < 30:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")

        # time difference
        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed

        # ------- Sensor space reading ------- 
        # Read sensor
        prox_res, prox_dist, prox_point, prox_obj, prox_n = sim.readProximitySensor(s3_Handle)
        if (prox_res): # prox_dist>0.1 and prox_dist< 5
            sensor_reading = np.array([
                [0],
                [0],
                [prox_dist],
                [1]
            ])
            z_sensor.append(prox_dist)
            t.append(elapsed)

            # ------- Body space reading ------- 
            # Use the object's 3x4 matrix (row-major) to build a robust 4x4 transform
            s3_mat = sim.getObjectMatrix(s3_Handle, p3dx)
            T_sensor_body = np.array([
                [s3_mat[0], s3_mat[1], s3_mat[2], s3_mat[3]],
                [s3_mat[4], s3_mat[5], s3_mat[6], s3_mat[7]],
                [s3_mat[8], s3_mat[9], s3_mat[10], s3_mat[11]],
                [0, 0, 0, 1]
            ])

            # Proximity sensors on the Pioneer are mounted pointing along their local X axis.
            # Build the local sensor-point (distance along X) and transform to body frame.
            sensor_point_local = np.array([
                [prox_dist],
                [0],
                [0],
                [1]
            ])
            sensor_reading_body = np.matmul(T_sensor_body, sensor_point_local)
            x_sensor_to_body.append(sensor_reading_body[0][0])
            y_sensor_to_body.append(sensor_reading_body[1][0])

            # ------- World space reading ------- 
            p3dx_mat = sim.getObjectMatrix(p3dx, sim.handle_world)
            T_body_world = np.array([
                [p3dx_mat[0], p3dx_mat[1], p3dx_mat[2], p3dx_mat[3]],
                [p3dx_mat[4], p3dx_mat[5], p3dx_mat[6], p3dx_mat[7]],
                [p3dx_mat[8], p3dx_mat[9], p3dx_mat[10], p3dx_mat[11]],
                [0, 0, 0, 1]
            ])
            sensor_reading_world = np.matmul(T_body_world, sensor_reading_body)
            x_sensor_to_world.append(sensor_reading_world[0][0])
            y_sensor_to_world.append(sensor_reading_world[1][0])

        

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")

#plot sensor overtime
plt.figure()
plt.plot(t, z_sensor, '.')
plt.xlabel('Time')
plt.ylabel('Sensor Reading')
plt.title('Ultrasonic Sensor Readings Over Time')
plt.show()

#plot sensor readings in body frame
plt.figure()
plt.plot(x_sensor_to_body, y_sensor_to_body, '.')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Sensor Readings in Body Frame')
# equal axis
plt.axis('equal')
plt.show()

#plot sensor readings in body frame overtime
plt.figure()
plt.plot(t, x_sensor_to_body, '.', label='X Position')
plt.plot(t, y_sensor_to_body, '.', label='Y Position')
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Sensor Readings in Body Frame Over Time')
plt.legend()
plt.show()

#plot sensor readings in world frame
plt.figure()
plt.plot(x_sensor_to_world, y_sensor_to_world, '.')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Sensor Readings in World Frame')
# equal axis
plt.axis('equal')
plt.show()