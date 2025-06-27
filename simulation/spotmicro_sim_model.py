from dataclasses import dataclass
from simulation_entity_base import SimulationEntityBase
import pybullet
import os

_URDF_PATH = os.path.dirname(__file__) + "/robot_model/spot.xml"

_VAR_INDEX_TO_JOINT_NAME = {
    0: "front_left_shoulder",
    1: "front_left_leg",
    2: "front_left_foot",
    3: "front_right_shoulder",
    4: "front_right_leg",
    5: "front_right_foot",
    6: "rear_left_shoulder",
    7: "rear_left_leg",
    8: "rear_left_foot",
    9: "rear_right_shoulder",
    10: "rear_right_leg",
    11: "rear_right_foot",
}


@dataclass
class BaseState:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]


class SpotMicroSimModel(SimulationEntityBase):
    def __init__(self, name):
        super().__init__(name)
        self.__control_input = []
        self.__control_mode = pybullet.POSITION_CONTROL
        self.__joint_mapping = {}

    def load(self) -> None:
        self._id = pybullet.loadURDF(
            fileName=_URDF_PATH,
            useFixedBase=False,
            useMaximalCoordinates=False,
            flags=pybullet.URDF_USE_SELF_COLLISION,
        )
        self.__control_input = []
        self.__control_mode = pybullet.POSITION_CONTROL
        pybullet.changeDynamics(self._id, -1, lateralFriction=0.8)

        self.__joint_mapping = self._get_joint_mapping()
        for _, joint_name in _VAR_INDEX_TO_JOINT_NAME.items():
            if joint_name not in self.__joint_mapping:
                raise ValueError(
                    f"Joint name '{joint_name}' not found in the agent's joint mapping."
                )

    @property
    def control_mapping(self) -> dict[int, str]:
        return _VAR_INDEX_TO_JOINT_NAME

    def _get_joint_mapping(self) -> dict[str, int]:
        nJoints = pybullet.getNumJoints(self._id)
        joint_mapping = {}

        for i in range(nJoints):
            jointInfo = pybullet.getJointInfo(self._id, i)
            joint_mapping[jointInfo[1].decode("UTF-8")] = jointInfo[0]
        return joint_mapping

    def set_control(
        self, control_input: list[float], control_mode=pybullet.POSITION_CONTROL
    ) -> None:
        if len(control_input) != len(_VAR_INDEX_TO_JOINT_NAME):
            raise ValueError(
                f"Control input must have {len(_VAR_INDEX_TO_JOINT_NAME)} elements."
            )
        self.__control_input = control_input
        self.__control_mode = control_mode

    def apply_control(self) -> None:
        if not self.__control_input:
            raise ValueError(
                "Control input is not set. Please set control input first."
            )

        for var_index in range(len(self.__control_input)):
            joint_name = _VAR_INDEX_TO_JOINT_NAME[var_index]
            joint_index = self.__joint_mapping[joint_name]
            pybullet.setJointMotorControl2(
                bodyIndex=self._id,
                jointIndex=joint_index,
                controlMode=self.__control_mode,
                targetPosition=self.__control_input[var_index],
            )

    def get_joint_states(self) -> list[tuple]:
        joint_states = []
        for var_index in range(len(_VAR_INDEX_TO_JOINT_NAME)):
            joint_name = _VAR_INDEX_TO_JOINT_NAME[var_index]
            joint_index = self.__joint_mapping[joint_name]
            joint_state = pybullet.getJointState(self._id, joint_index)
            joint_states.append(joint_state)
        return joint_states

    def get_base_state(self) -> BaseState:
        base_position, base_orientation = pybullet.getBasePositionAndOrientation(
            self.__agent
        )
        base_linear_velocity, base_angular_velocity = pybullet.getBaseVelocity(
            self.__agent
        )
        return BaseState(
            position=base_position,
            orientation=base_orientation,
            linear_velocity=base_linear_velocity,
            angular_velocity=base_angular_velocity,
        )

    def step(self):
        self.apply_control()
