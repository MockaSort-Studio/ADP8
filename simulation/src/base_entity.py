from abc import ABC


class BaseEntity(ABC):
    def __init__(self, name: str):
        self.name = name
        self.gs_entity = None

    def add_to_scene(self, scene):
        raise RuntimeError("add_to_scene must be implemented!")

    def get_observation(self):
        raise RuntimeError("get_observation must be implemented!")

    def get_groundtruth(self):
        raise RuntimeError("get_groundtruth must be implemented!")

    def step(self):
        raise RuntimeError("step must be implemented!")
