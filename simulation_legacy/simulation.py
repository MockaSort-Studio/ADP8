import pybullet_data
import pybullet
import time
import simulation_scenario
import simulation_agent
import json
from os import path
import logging

CONFIGURATION_KEYS = [
    "use_real_time",
    "physics_engine_connection",
    "max_physics_solver_iterations",
]


class SimulationParameters:
    def __init__(self, config_path="", default=True):
        self.valid = False
        if default:
            self.use_real_time = True
            self.physics_engine_connection = pybullet.SHARED_MEMORY
            self.max_physics_solver_iterations = 200
            self.valid = True
        else:
            self._parse_config_file(config_path)

    def _parse_config_file(self, config_file_path):
        if not path.isfile(config_file_path):
            return
        file = open(config_file_path, "r")

        json_dict = dict()
        try:
            json_dict = json.load(file)
        except ValueError:
            return

        if not self._json_dict_valid(json_dict):
            return
        self.use_real_time = json_dict[CONFIGURATION_KEYS[0]]
        self.physics_engine_connection = json_dict[CONFIGURATION_KEYS[1]]
        self.max_physics_solver_iterations = json_dict[CONFIGURATION_KEYS[2]]
        self.valid = True

    def _json_dict_valid(self, json_dict):
        return all(key in CONFIGURATION_KEYS for key in json_dict)


# leaving default pybullet step size
FIXED_TIME_STEP_SECONDS = 0.0041


class SimulationClient:
    def __init__(self, parameters: SimulationParameters):
        self.parameters = parameters
        self.agent = None
        self.scenario = None

    def _connect_to_physics_server(self, physics_engine_connection_type):
        self.engine_client_id = pybullet.connect(physics_engine_connection_type)
        if self.engine_client_id < 0:
            self.engine_client_id = pybullet.connect(pybullet.GUI)

    def _setup_simulation_time(self, use_real_time, physics_engine_connection_type):
        self.use_real_time = use_real_time
        self.reference_time = time.time()
        self.current_time = 0
        if physics_engine_connection_type == pybullet.DIRECT:
            self.use_real_time = False
        pybullet.setRealTimeSimulation(self.use_real_time)

    def _setup_simulation_client(self):
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 1)

    def _setup_physics_engine(self, max_solver_iterations, cycle_step_seconds):
        pybullet.setPhysicsEngineParameter(
            numSolverIterations=max_solver_iterations, fixedTimeStep=cycle_step_seconds
        )
        pybullet.setGravity(0, 0, -9.81)

    def _update_internal_timestamp(self):
        if self.use_real_time:
            self.current_time = time.time() - self.reference_time
        else:
            self.current_time += FIXED_TIME_STEP_SECONDS

    def reset(self):
        if pybullet.isConnected():
            pybullet.resetSimulation()

    def set_simulated_agent(self, agent: simulation_agent.SimulationAgent):
        if not type(agent) == simulation_agent.SimulationAgent:
            TypeError(
                "Cannot load agent, SimulationAgent has to be used to create agents"
            )
        self.agent = agent
        self.agent.load()
        pass

    def set_simulation_scenario(
        self, scenario: simulation_scenario.BaseSimulationScenario
    ):
        if not issubclass(type(scenario), simulation_scenario.BaseSimulationScenario):
            TypeError(
                "Cannot load scenario, BaseSimulationScenario has to be used to create scenarios"
            )
        logging.info("Loading Scenario {}".format(scenario.name))
        self.scenario = scenario
        self.scenario.load()
        logging.info("Scenario {} successfully loaded".format(scenario.name))

    def start(self):
        self.engine_client_id = self._connect_to_physics_server(
            self.parameters.physics_engine_connection
        )
        self._setup_simulation_time(
            self.parameters.use_real_time, self.parameters.physics_engine_connection
        )
        # pybullet configuration
        self._setup_simulation_client()
        # physics engine configuration
        self._setup_physics_engine(
            self.parameters.max_physics_solver_iterations, FIXED_TIME_STEP_SECONDS
        )

    def step(self):
        self._update_internal_timestamp()
        self.scenario.step()
        self.agent.step()
        if not self.use_real_time:
            pybullet.stepSimulation()

    def shutdown(self):
        pybullet.disconnect()
