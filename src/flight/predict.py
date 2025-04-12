from dataclasses import dataclass
import logging
import traceback

import numpy as np

from base.component import Component
from base.stage import Stage
from flight.blackboard import FilterState, FusionState, PredictState, StageState, ControlState
from simulation.dynamics import calculate_derivatives
from simulation.vehicle import Vehicle
from simulation.motor import Motor
from simulation.environment import Environment

logger = logging.getLogger(__name__)


class PredictComponent(Component):

    def __init__(self, filter_state: FilterState, stage_state: StageState,
            fusion_state: FusionState, control_state: ControlState, vehicle: Vehicle, motor: Motor,
                 environment: Environment):
        self._state = PredictState()

        self._filter_state = filter_state
        self._stage_state = stage_state
        self._fusion_state = fusion_state
        self._control_state = control_state

        self._vehicle = vehicle
        self._motor = motor
        self._environment = environment

        logger.info("Prediction Component Initialized.")

    @property
    def state(self):
        return self._state

    def dispatch(self, time: float):

        try:
            if self._stage_state.stage not in [
                    Stage.COAST, Stage.OVERSHOOT, "COAST"
            ]:
                return
            mass = self._vehicle.mass + self._motor.calculate_mass(time)
            position = np.array([0, 0, self._filter_state.altitude])
            linear_momentum = np.array(self._filter_state.velocity) * mass
            orientation = np.array([
                self._fusion_state.quaternion[3],
                self._fusion_state.quaternion[0],
                self._fusion_state.quaternion[1],
                self._fusion_state.quaternion[2]
            ])
            angular_momentum = np.array([0, 0, 0], dtype=np.float64)
            vehicle = self._vehicle
            motor = self._motor
            environment = self._environment
            angle_of_actuation = self._control_state.servo_angle
            
            time_delta = np.float64(1)

            while float(linear_momentum[2]) > 1:
                # Perform RK4.
                k1p, k1l, k1o, k1a = calculate_derivatives(
                    vehicle, motor, environment, angle_of_actuation, time, position,
                    linear_momentum, orientation, angular_momentum)
                k2p, k2l, k2o, k2a = calculate_derivatives(
                    vehicle, motor, environment, angle_of_actuation, time + time_delta / 2,
                    position + time_delta * k1p / 2,
                    linear_momentum + time_delta * k1l / 2,
                    orientation + time_delta * k1o / 2,
                    angular_momentum + time_delta * k1a / 2)
                k3p, k3l, k3o, k3a = calculate_derivatives(
                    vehicle, motor, environment, angle_of_actuation, time + time_delta / 2,
                    position + time_delta * k2p / 2,
                    linear_momentum + time_delta * k2l / 2,
                    orientation + time_delta * k2o / 2,
                    angular_momentum + time_delta * k2a / 2)
                k4p, k4l, k4o, k4a = calculate_derivatives(
                    vehicle, motor, environment, angle_of_actuation, time + time_delta,
                    position + time_delta * k3p,
                    linear_momentum + time_delta * k3l,
                    orientation + time_delta * k3o,
                    angular_momentum + time_delta * k3a)
                position += time_delta / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
                linear_momentum += time_delta / 6 * (k1l + 2 * k2l + 2 * k3l +
                                                     k4l)
                orientation += time_delta / 6 * (k1o + 2 * k2o + 2 * k3o + k4o)
                angular_momentum += time_delta / 6 * (k1a + 2 * k2a + 2 * k3a +
                                                      k4a)

            self._state.apogee_prediction = float(position[2])

        except Exception as e:
            logger.info(f"Exception when predicting: {traceback.format_exc()}")
