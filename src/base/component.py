from abc import ABC, abstractmethod


class Component(ABC):

    @abstractmethod
    def dispatch(self, time: float):
        pass
