from simulation.src.simulation_runner import SimulationRunner
from simulation.agents.rc_car import RCCar


if __name__ == "__main__":
    rccar = RCCar()
    sim = SimulationRunner(agent=rccar, show_viewer=False)

    for _ in range(100):
        obs = sim.get_observations()
        print(f"imu data: {obs}")
        sim.step(inputs=[0, 0])
