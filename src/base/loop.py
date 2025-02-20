from dataclasses import dataclass, field
import logging
import time

from base.component import Component

logger = logging.getLogger(__name__)


@dataclass
class LoopState:
    frequency: int = 0
    period: float = 0

    components: list = field(default_factory=list)

    first_time: float = 0
    time: float = 0
    step_count: int = 0
    slip_count: int = 0

    # How often altimeter gets zeroed on Ground, in seconds
    zero_period: int = 60*10


class Loop:

    def __init__(self, frequency: int):
        assert frequency > 0

        self._state = LoopState()
        self._state.frequency = frequency
        self._state.period = 1 / frequency

        logger.info("Loop initialized.")
        logger.info(f"{self._state.frequency=}")
        logger.info(f"{self._state.period=}")

    @property
    def state(self):
        return self._state

    def _step(self):
        for component, frequency in self._state.components:
            if self._state.step_count % (self._state.frequency //
                                         frequency) == 0:
                component.dispatch()

    def add_component(self, component: Component, frequency: int):
        assert frequency > 0
        assert self._state.frequency % frequency == 0

        logger.info(
            f"Adding component {type(component).__name__} with frequency {frequency} Hz."
        )
        self._state.components.append((component, frequency))

    def run(self, steps: int):
        iterator = range(steps) if steps > 0 else iter(int, 1)

        start = time.time()
        self._state.first_time = start
        for _ in iterator:
            self._state.time = start
            self._step()
            self._state.step_count += 1

            end = time.time()
            delta = end - start
            if delta > self._state.period:
                self._state.slip_count += 1
                start = end
            else:
                time.sleep(self._state.period - delta)
                start += self._state.period
