import logging
from typing import Optional

from base.loop import Loop
from simulation.dynamics import DynamicsComponent
from simulation.environment import Environment
from simulation.motor import Motor
from simulation.vehicle import Vehicle

logger = logging.getLogger(__name__)


class Simulation:

    def __init__(self, frequency: int, vehicle: Vehicle, motor: Motor,
                 environment: Environment):
        self.loop = Loop(frequency)
        loop_state = self.loop.state

        dynamics_component = DynamicsComponent(vehicle, motor, environment)
        dynamics_state = dynamics_component.state
        self.loop.add_component(dynamics_component, frequency)

        self.dynamics_state = dynamics_state

    def run(self):
        while self.dynamics_state.position[2] > -100:
            self.loop.run(1)
