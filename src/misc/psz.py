from dataclasses import dataclass

import traceback

import numpy as np

from base.component import Component
from base.stage import Stage

from flight.blackboard import FilterState, FusionState, ControlState, StageState, PredictState
from flight.constants import SIM_APOGEE
from simulation.vehicle import Vehicle
from simulation.motor import Motor
from simulation.environment import Environment

from base.constants import EARTH_GRAVITY_ACCELERATION
from simulation.constants import CFD_AIR_DENSITY

class PSZComponent(Component):

    def __init__(self, filter_state: FilterState, stage_state: StageState,
                 fusion_state: FusionState, control_state: ControlState,
                 vehicle: Vehicle, motor: Motor, environment: Environment):

        self._state = PredictState()

        self._filter_state = filter_state
        self._control_state = control_state
        self._stage_state = stage_state
        self._fusion_state = fusion_state
        
        self._vehicle = vehicle
        self._motor = motor
        self._environment = environment

    def state(self):
        return self._state


    def dispatch(self, time:float):

        try:
            if self._stage_state.stage not in [
                    Stage.COAST, Stage.OVERSHOOT, "COAST"
                    ]:
                return

            position = self._filter_state.altitude
            velocity = self._filter_state.velocity[2]
            
            vehicle = self._vehicle
            motor = self._motor
            environment = self._environment
            actuation_angle = self._control_state.servo_angle
            
            predict_apogee(

    def predict_apogee(self, actuation_angle, altitude, velocity, vehicle, motor, environment):

        dt = 0.5

        apogee_prediction = altitude
        velocity_current = velocity
        

        force_axial = -aerodynamic_multiplier * vehicle.calculate_axial_force(
            actuation_angle, 0, 

    def calculate_drag(self, actuation_angle, altitude, velocity, vehicle, motor, environment):

        air_pressure = environment.ground_pressure * (air_temperature / 518.6)**5.256
        air_temperature = environment.ground_temperature - 0.00356 * altitude
        air_density = air_pressure / 1718 / air_temperature

        mach_number = velocity / np.sqrt(1.4 * air_pressure / air_density)
        
        force_axial = 0
        if velocity != 0:

            force_axial = -(air_density / CFD_AIR_DENSITY) * vehicle.calculate_axial_force(
                actuation_angle, 0, mach_number)

        if force_axial <= 0:
            return 0

        return force_axial

        
