from abc import ABC


class BaseEntity(ABC):
    def __init__(self, name: str):
        self.name = name
        self.gs_entity = None

    def add_to_scene(self, scene) -> None:
        raise RuntimeError("add_to_scene must be implemented!")

    def get_observation(self) -> dict:
        raise RuntimeError("get_observation must be implemented!")

    def get_noisy_observation(self) -> dict:
        raise RuntimeError("get_noisy_observation must be implemented!")

    def step(self) -> None:
        raise RuntimeError("step must be implemented!")
