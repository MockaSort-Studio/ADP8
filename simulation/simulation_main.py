from simulation_parameters import SimulationParameters
from simulation_runner import SimulationRunner
from spotmicro_sim_model import SpotMicroSimModel
import os

DEFAULT_PARAMETER_FILE_PATH = os.path.join(
    os.path.dirname(__file__), "simulation_parameters.yaml"
)

if __name__ == "__main__":
    try:
        sim_parameters = SimulationParameters(DEFAULT_PARAMETER_FILE_PATH)
        print(f"Simulation Runtime Parameters: {sim_parameters.runtime_params}")
        sim_runner = SimulationRunner(sim_parameters)
        spotmicro = SpotMicroSimModel("Spot")
        sim_runner.start()
        sim_runner.set_simulated_entity(spotmicro)

    except KeyboardInterrupt:
        pass
    finally:
        sim_runner.shutdown()
