import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# Start Simulation
sim.startSimulation()
print("Simulation Started")

right_wheel = sim.getObject('/PioneerP3DX/rightMotor')
left_wheel = sim.getObject('/PioneerP3DX/leftMotor')

# Parameters
rw = 0.195 / 2
rb = 0.381 / 2
L = 0.381
d = 0.05

# Lists to store temporal data for plotting
t_data = []
wr_data = []
wl_data = []
vx_data = []
wx_data = []

try:
    # Main Loop (Run for 30 seconds)
    start_time = time.time()
    
    while True:
        current_time = time.time()
        elapsed = current_time - start_time
        
        if elapsed >= 30:
            break
            
        print(f"Running... {elapsed:.1f}s", end="\r")
        
        # Getting actual joint velocity for the plots
        wr_vel = sim.getJointVelocity(right_wheel)
        wl_vel = sim.getJointVelocity(left_wheel)

        # Somehow V_x is slight bigger than 2m/s, which is invalid by kinematic math
        # vx = (wr_vel + wl_vel) * rw / rb
        # wx = (wr_vel - wl_vel) * rw / rb
        # New calculation based on the kinematic model of a differential drive robot
        vx = (rw / 2.0) * (wr_vel + wl_vel)
        wx = (rw / L) * (wr_vel - wl_vel)

        # Append data to lists
        t_data.append(elapsed)
        wr_data.append(wr_vel)
        wl_data.append(wl_vel)
        vx_data.append(vx)
        wx_data.append(wx)

        # time.sleep(0.05) 

finally:
    # Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")
    
    # Plotting the Results
    plt.figure(figsize=(10, 8))
    
    # Subplot 1: Temporal plot of P3DX joint velocity
    plt.subplot(2, 1, 1)
    plt.plot(t_data, wr_data, label=r'$\dot{\varphi}_R$ (Right Wheel)', color='blue')
    plt.plot(t_data, wl_data, label=r'$\dot{\varphi}_L$ (Left Wheel)', color='red', linestyle='dashed')
    plt.title('Temporal Plot of P3DX Joint Velocity')
    plt.xlabel('t (sec)')
    plt.ylabel('Velocity (rad/s)')
    plt.grid(True)
    plt.legend()
    
    # Subplot 2: Temporal plot of P3DX body velocity
    plt.subplot(2, 1, 2)
    plt.plot(t_data, vx_data, label=r'$V_x$ (Linear Velocity)', color='green')
    plt.plot(t_data, wx_data, label=r'$\omega$ (Angular Velocity)', color='purple', linestyle='dashed')
    plt.title('Temporal Plot of P3DX Body Velocity')
    plt.xlabel('t (sec)')
    plt.ylabel('Velocity')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.show()