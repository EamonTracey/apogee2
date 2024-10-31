from component import Component
from loop import Loop

class Flight:
    def __init__(self):
        self.loop = Loop()

    def fly(self):
        self.loop.run(-1)
