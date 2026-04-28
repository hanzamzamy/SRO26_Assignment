import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from robot_roles import StrikerRobot, GoalkeeperRobot, PasserRobot, GameState

def main():
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)

    print("Initializing Robots...")
    striker = StrikerRobot(sim, robot_name='Robot_Pemain', goal_name='Gwang_Kuning', red_ball='Bola_Merah', blue_ball='Bola_Biru', gk_name='Robot_Lawan_01')
    goalkeeper = GoalkeeperRobot(sim, robot_name='Robot_Lawan_01', goal_name='Gwang_Kuning')
    passer = PasserRobot(sim, robot_name='Robot_Lawan_02', blue_ball='Bola_Biru')

    # Get handles to check for goals
    goal_handle = sim.getObject('/Gwang_Kuning')
    red_ball = sim.getObject('/Bola_Merah')
    blue_ball = sim.getObject('/Bola_Biru')

    print("Starting Simulation...")
    sim.startSimulation()

    try:
        run_time = 90.0
        while True:
            t = sim.getSimulationTime()
            if t >= run_time:
                break
                
            # --- GOAL DETECTION LOGIC ---
            goal_pos = sim.getObjectPosition(goal_handle, sim.handle_world)
            red_pos = sim.getObjectPosition(red_ball, sim.handle_world)
            blue_pos = sim.getObjectPosition(blue_ball, sim.handle_world)

            # The goal origin is at the edge.
            goal_center_y = 0.0

            # Check if Red Ball is inside/very close to the Goal
            if not GameState.red_goal_scored and math.hypot(red_pos[0] - goal_pos[0], red_pos[1] - goal_center_y) < 1.2:
                print("\n*** GOAL! Bola Merah Masuk! ***")
                GameState.red_goal_scored = True
                GameState.active_ball_name = 'Bola_Biru' # GK is allowed to track the Blue ball

            # Check if Blue Ball is inside/very close to the Goal
            if GameState.red_goal_scored and math.hypot(blue_pos[0] - goal_pos[0], blue_pos[1] - goal_center_y) < 1.2:
                print("\n*** GOAL! Bola Biru Masuk! ***")
                break # Terminate simulation upon second goal

            print(f"Time: {t:.1f}s | Striker: {striker.state} | Passer: {passer.state} | GK: {goalkeeper.state}      ", end="\r")

            striker.step()
            goalkeeper.step()
            passer.step()

            sim.step()

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        # Prevent ZMQ from hanging by disabling synchronous stepping before stopping
        sim.setStepping(False)
        time.sleep(0.1) 
        sim.stopSimulation()
        print("\nSimulation Stopped.")

if __name__ == '__main__':
    main()