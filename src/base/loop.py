from dataclasses import dataclass, field
import logging
import time

from base.component import Component

logger = logging.getLogger(__name__)


@dataclass
class LoopState:
    # Frequency in Hertz.
    frequency: int = 0
    # Period in seconds.
    period: float = 0

    components: list = field(default_factory=list)

    # First Unix epoch time in seconds.
    first_time: float = 0
    # Last Unix epoch time in seconds.
    time: float = 0

    step_count: int = 0
    slip_count: int = 0


class Loop:

    def __init__(self, frequency: int, *, real_time: bool = True):
        assert frequency > 0

        self._state = LoopState()
        self._state.frequency = frequency
        self._state.period = 1 / frequency

        self._real_time = real_time

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
                component.dispatch(self._state.time - self._state.first_time)

    def add_component(self, component: Component, frequency: int):
        assert frequency > 0
        assert self._state.frequency % frequency == 0

        logger.info(
            f"Adding component {type(component).__name__} with frequency {frequency} Hz."
        )
        self._state.components.append((component, frequency))

    def run(self, steps: int):
        start: float
        if self._real_time:
            start = time.time()
        else:
            start = self._state.time + self._state.period

        if self._state.first_time == 0:
            self._state.first_time = start

        iterator = range(steps) if steps > 0 else iter(int, 1)
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
                if self._real_time:
                    time.sleep(self._state.period - delta)
                start += self._state.period
