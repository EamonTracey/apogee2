import logging 

from base.loop import Loop

from flight.predict import PredictComponent
from replay_flight.data_replay import DataReplayComponent
from replay_flight.log_replay import LogReplayComponent

from simulation.environment import Environment
from simulation.vehicle import Vehicle
from simulation.motor import Motor

logger = logging.getLogger(__name__)

class Replay:

    def __init__(self, name: str, vehicle: Vehicle, motor: Motor,
                 environment: Environment, path: str):

        self.loop = Loop(30)
        loop_state = self.loop.state
        
        # Replay flight data
        replay_data_component = DataReplayComponent(path)
        filter_state = replay_data_component.get_filter_state
        fusion_state = replay_data_component.get_fusion_state
        stage_state = replay_data_component.get_stage_state
        control_state = replay_data_component.get_control_state
        self.loop.add_component(replay_data_component, 30)

        # Predict Apogee
        predict_component = PredictComponent(filter_state, stage_state, 
                                             fusion_state, control_state, vehicle, motor, environment)
        predict_state = predict_component.state
        self.loop.add_component(predict_component, 30)

        # Log
        log_path = f"{name}.csv"
        log_replay_component = LogReplayComponent(log_path, loop_state,
                                                  filter_state, control_state,
                                                  fusion_state, predict_state,
                                                  stage_state)
        self.loop.add_component(log_replay_component, 30)

    def run(self):
        self.loop.run(0)






