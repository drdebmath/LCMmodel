from enum import Enum


class RobotState(str, Enum):
    LOOK = "LOOK"
    MOVE = "MOVE"
    WAIT = "WAIT"
    TERMINATED = "TERMINATED"

    def next_state(self):
        if self == RobotState.LOOK:
            return RobotState.MOVE
        elif self == RobotState.MOVE:
            return RobotState.WAIT
        elif self == RobotState.WAIT:
            return RobotState.LOOK


class SchedulerType(Enum):
    ASYNC = "Async"


class DistributionType(Enum):
    EXPONENTIAL = "Exponential"


class Algorithm(Enum):
    GATHERING = "Gathering"
    SEC = "SEC"
    CUSTOM = "Custom"
