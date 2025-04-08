import logging
from dataclasses import dataclass

from base.component import Component
from base.stage import Stage
from flight.blackboard import FilterState, StageState, FusionState

import pandas as pd
import numpy as np

class TestComponent(Component): 
    def __init__(self):
        self._filter_state = FilterState()
        self._stage_state = StageState()
        self._fusion_state = FusionState()

        filepath = '/home/acs/apogee2/data/launches/fullscale2/ACS_20250406191454.csv'
        
        df = pd.read_csv(filepath)
        self._data = df.iloc[29090:29290].to_numpy()
        print(self._data)

        self.i = 0

        self.time = 0

        
    @property 
    def get_filter_state(self):
        return self._filter_state
    
    @property
    def get_stage_state(self):
        return self._stage_state
    
    @property
    def get_fusion_state(self):
        return self._fusion_state

   

    def dispatch(self, time: float):
    
        self._filter_state.altitude = self._data[self.i][5]
        self._filter_state.velocity = (self._data[self.i][6], self._data[self.i][7], self._data[self.i][8])
        self._filter_state.acceleration = (self._data[self.i][9], self._data[self.i][10], self._data[self.i][11])

        self._stage_state.stage = self._data[self.i][2]

        self._fusion_state.quaternion = (self._data[self.i][33],self._data[self.i][34],self._data[self.i][35],self._data[self.i][36])
        self._fusion_state.euler = (self._data[self.i][37],self._data[self.i][38],self._data[self.i][39])
        self._fusion_state.zenith = self._data[self.i][40]

        self.time = self._data[self.i][0]

        self.i = self.i + 1
        


