from dataclasses import dataclass
import board
import logging
import busio
import adafruit_bmp3xx
import adafruit_bmp3xx.i2c
from datetime import datetime

from base.component import Component

@dataclass
class BMP390State:
    # variables
    pressure: float = 0
    altitude: float = 0
    temperature: float = 0
    
    # errors 
    pressure_errors: int = 0
    altitude_errors: int = 0
    temperature_errors: int = 0

class BMP390Component(Component):
    def __init__(self, i2c: busio.I2C, address: int = 0x4a): # placeholder address - update later
        self._state = BMP390State()
        
        self._bmp390 = adafruit_bmp3xx.BMP3XX_I2C(i2c)

    @property
    def state(self):
        return self._state

    def dispatch(self):
        altitude = None
        try:
            altitude = self._bmp390.altitude
        except Exception as e:
            logging.exception(f"Error reading the BMP390 altimeter: {e}.")
        if altitude is not None:
            self._state.altitude = altitude
        else:
            self._state.altitude_errors += 1
            
        pressure = None
        try:
            pressure = self._bmp390.pressure
        except Exception as e:
            logging.exception(f"Error reading the BMP390 altimeter: {e}.")
        if pressure is not None:
            self._state.pressure = pressure
        else:
            self._state.pressure_errors += 1
            
        temperature = None
        try:
            temperature = self._bmp390.temperature
        except Exception as e:
            logging.exception(f"Error reading the BMP390 altimeter: {e}.")
        if temperature is not None:
            self._state.temperature = temperature
        else:
            self._state.temperature_errors += 1