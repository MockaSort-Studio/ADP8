import pybullet_data
import pybullet
import abc


class SimulationAgent(abc.ABC):
    def __init__(self, name, agent_description_file_path) -> None:
        self.__name = name
        self.__agent = None
        self.__agent_description = agent_description_file_path

    @property
    def name(self):
        return self.__name

    @property
    def agent(self):
        return self.__agent

    # Fix coordinates
    def load(self) -> None:
        self.__agent = pybullet.loadURDF(
            self.__agent_description,
            # pybullet.getQuaternionFromEuler([0, 0, 90.0]),
            # [0, 0, 0.3],
            useFixedBase=False,
            useMaximalCoordinates=False,
            flags=pybullet.URDF_USE_SELF_COLLISION,
        )
        pybullet.changeDynamics(self.__agent, -1, lateralFriction=0.8)

    def step(self):
        pass
