import logging
from typing import Optional

import board
import busio

from base.loop import Loop
from flight.bmp390 import BMP390Component
from flight.bno085 import BNO085Component
from flight.control import ControlComponent
from flight.filter import FilterComponent
from flight.icm20649 import ICM20649Component
from flight.log import LogComponent
from flight.stage import StageComponent
from flight.predict import PredictComponent
from flight.fusion import FusionComponent
from simulation.environment import Environment
from simulation.vehicle import Vehicle
from simulation.motor import Motor

logger = logging.getLogger(__name__)


class Flight:

    def __init__(self, name: str, vehicle: Vehicle, motor: Motor,
                 environment: Environment):
        self.loop = Loop(30)
        loop_state = self.loop.state

        # Connect to the I2C bus.
        i2c = busio.I2C(board.SCL, board.SDA)

        # BMP390.
        bmp390_component = BMP390Component(i2c)
        bmp390_state = bmp390_component.state
        self.loop.add_component(bmp390_component, 30)

        # BNO085.
        bno085_component = BNO085Component(i2c)
        bno085_state = bno085_component.state
        self.loop.add_component(bno085_component, 30)

        # ICM20649.
        icm20649_component = ICM20649Component(i2c)
        icm20649_state = icm20649_component.state
        self.loop.add_component(icm20649_component, 30)

        # Filter.
        filter_component = FilterComponent(loop_state, bmp390_state,
                                           icm20649_state, bno085_state, None,
                                           None)
        filter_state = filter_component.state
        self.loop.add_component(filter_component, 30)

        # Stage Determination.
        stage_component = StageComponent(filter_state)
        stage_state = stage_component.state
        self.loop.add_component(stage_component, 30)

        ### CRAZY ###
        filter_component._stage_state = stage_state
        ### CRAZY ###

        # Fusion.
        fusion_component = FusionComponent(bno085_state, stage_state)
        fusion_state = fusion_component.state
        self.loop.add_component(fusion_component, 30)

        ### CRAZY (pt 2) ###
        filter_component._fusion_state = fusion_state
        ### CRAZY (pt 2) ###

        # Apogee Prediction.
        predict_component = PredictComponent(filter_state, stage_state,
                                             fusion_state, None, vehicle,
                                             motor, environment)
        predict_state = predict_component.state
        self.loop.add_component(predict_component, 30)

        # Motor Control.
        control_component = ControlComponent(filter_state, stage_state,
                                             predict_state)
        control_state = control_component.state
        self.loop.add_component(control_component, 30)

        ### CRAZY (pt 3) ###
        predict_component._control_state = control_state
        ### CRAZY (pt 3) ###

        # Log.
        log_path = f"{name}.csv"
        log_component = LogComponent(log_path, loop_state, bmp390_state,
                                     bno085_state, icm20649_state,
                                     filter_state, control_state, stage_state,
                                     predict_state, fusion_state)
        self.loop.add_component(log_component, 30)

    def run(self):
        self.loop.run(0)
