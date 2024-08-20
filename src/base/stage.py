from enum import Enum


class Stage(Enum):
    """The stages of rocket flight."""

    GROUND = 0
    RAIL = 1
    BURN = 2
    COAST = 3
    DESCENT = 4
