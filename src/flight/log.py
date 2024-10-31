from dataclasses import dataclass

from base.component import Component

@dataclass
class LogState:
    ...

class LogComponent(Component):
    def __init__(self, log_file: str):
        self._state = LogState()

        self._log_file = open(log_file, "w")

    def dispatch(self):
        # self._log_file.write(...)

