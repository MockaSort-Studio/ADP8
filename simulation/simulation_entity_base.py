from abc import ABC, abstractmethod


class SimulationEntityBase(ABC):
    def __init__(self, name):
        self.__name = name
        self._id = None

    @property
    def name(self) -> str:
        return self.__name

    @property
    def id(self) -> int:
        return self._id

    @abstractmethod
    def load(self):
        pass

    @abstractmethod
    def step(self):
        pass
