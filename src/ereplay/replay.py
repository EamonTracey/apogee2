import logging

import pandas as pd

from base.loop import Loop, ReplayLoop
from ereplay.data_replay import DataReplayComponent
from flight.filter import FilterComponent
from flight.fusion import FusionComponent
from flight.predict import PredictComponent
from flight.stage import StageComponent
from simulation.environment import Environment
from simulation.vehicle import Vehicle
from simulation.motor import Motor

logger = logging.getLogger(__name__)


class Replay:

    def __init__(self, name: str, vehicle: Vehicle, motor: Motor,
                 environment: Environment, path: str):
        times = pd.read_csv(path)["Time"].to_list()

        self.loop = ReplayLoop(30, times)
        loop_state = self.loop.state

        # Data replay.
        data_replay_component = DataReplayComponent(path)
        bmp390_state = data_replay_component.bmp390_state
        bno085_state = data_replay_component.bno085_state
        icm20649_state = data_replay_component.icm20649_state
        control_state = data_replay_component.control_state
        self.loop.add_component(data_replay_component, 30)

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

        # Fusion.
        fusion_component = FusionComponent(bno085_state, stage_state)
        fusion_state = fusion_component.state
        self.loop.add_component(fusion_component, 30)

        ### CRAZY ###
        filter_component._stage_state = stage_state
        filter_component._fusion_state = fusion_state
        ### CRAZY ###

        # Apogee Prediction.
        predict_component = PredictComponent(filter_state, stage_state,
                                             fusion_state, control_state, vehicle,
                                             motor, environment)
        predict_state = predict_component.state
        self.loop.add_component(predict_component, 30)

        # TODO: log

    def run(self):
        self.loop.run(0)
