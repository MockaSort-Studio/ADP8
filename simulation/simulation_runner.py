import pybullet
from simulation_parameters import SimulationParameters
from simulation_entity_base import SimulationEntityBase
import time


class SimulationRunner:
    def __init__(self, parameters: SimulationParameters):
        self.parameters = parameters
        self.simulated_entities = []  # Unified list for agent, scenario, etc.

    def start(self):
        self.engine_client_id = pybullet.connect(
            self.parameters.physics_engine_connection
        )
        pybullet.setRealTimeSimulation(self.parameters.runtime_params.real_time)
        self.reference_time = time.time()
        pybullet.setPhysicsEngineParameter(
            numSolverIterations=self.parameters.runtime_params.max_physics_solver_iterations,
            fixedTimeStep=self.parameters.runtime_params.time_step,
        )
        gravity_vector = self.parameters.runtime_params.gravity_vector
        pybullet.setGravity(*gravity_vector)

    def set_simulated_entity(self, entity: SimulationEntityBase):
        if not isinstance(entity, SimulationEntityBase):
            raise TypeError(
                "entity must be an instance of SimulationEntityBase or its subclass"
            )
        entity.load()
        self.simulated_entities.append(entity)

    def step_simulation(self):
        for entity in self.simulated_entities:
            entity.step()
        if not self.parameters.runtime_params.real_time:
            pybullet.stepSimulation()

    def shutdown(self):
        pybullet.disconnect()
