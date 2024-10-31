from dataclasses import dataclass, field
import time

from component import Component


@dataclass
class LoopState:
    frequency: int = 0
    period: float = 0

    components: list = field(default_factory=list)

    step_count: int = 0
    slip_count: int = 0


class Loop:

    def __init__(self, frequency: int):
        assert frequency > 0

        self._state = LoopState()
        self._state.frequency = frequency
        self._state.period = 1 / frequency

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

        self._state.components.append((component, frequency))

    def run(self, steps: int):
        iterator = range(steps) if steps > 0 else iter(int, 1)

        start = time.time()
        for _ in iterator:
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
