import logging
from typing import Optional

from base.loop import Loop
from simulation.dynamics import DynamicsComponent
from simulation.environment import Environment
from simulation.log import LogComponent
from simulation.motor import Motor
from simulation.vehicle import Vehicle

logger = logging.getLogger(__name__)


class Simulation:

    def __init__(self, name: str, frequency: int, vehicle: Vehicle,
                 motor: Motor, environment: Environment, zenith: float,
                 azimuth: float):
        self.loop = Loop(frequency, real_time=False)
        loop_state = self.loop.state

        dynamics_component = DynamicsComponent(vehicle, motor, environment,
                                               zenith, azimuth)
        dynamics_state = dynamics_component.state
        self.loop.add_component(dynamics_component, frequency)

        log_component = LogComponent(f"{name}.csv", dynamics_state)
        log_state = log_component.state
        self.loop.add_component(log_component, frequency)

        self.dynamics_state = dynamics_state

    def run(self):
        while self.dynamics_state.position[2] > -1:
            self.loop.run(1)
