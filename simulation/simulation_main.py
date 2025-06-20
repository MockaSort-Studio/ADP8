from simulation_parameters import SimulationParameters
import os
import pybullet


DEFAULT_PARAMETER_FILE_PATH = os.path.join(
    os.path.dirname(__file__), "simulation_parameters.yaml"
)

if __name__ == "__main__":
    sim_parameters = SimulationParameters(DEFAULT_PARAMETER_FILE_PATH)
