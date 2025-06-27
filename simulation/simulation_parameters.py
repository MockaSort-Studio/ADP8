import yaml
from dataclasses import dataclass, fields
import pybullet


@dataclass
class RuntimeParams:
    real_time: bool
    time_step: float
    max_physics_solver_iterations: int
    gravity_vector: tuple[float, float, float]


class SimulationParameters:
    def __init__(self, parameter_file_path):
        self.physics_engine_connection = pybullet.SHARED_MEMORY
        self.runtime_params: RuntimeParams
        self._parse_parameter_file(parameter_file_path)

    def _parse_parameter_file(self, parameter_file_path):
        with open(parameter_file_path, "r") as file:
            params = yaml.safe_load(file)
            for field in fields(RuntimeParams):
                if field.name not in params:
                    raise KeyError(
                        f"Parameter file {parameter_file_path}: Missing required parameter: {field.name}"
                    )
            # Filter out extra keys
            filtered_params = {
                field.name: params[field.name] for field in fields(RuntimeParams)
            }
            self.runtime_params = RuntimeParams(**filtered_params)
