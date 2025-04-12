from enum import Enum


class RobotState(str, Enum):
    LOOK = "LOOK"
    MOVE = "MOVE"
    WAIT = "WAIT"
    TERMINATED = "TERMINATED"
    CRASH = "CRASH"

    def next_state(self):
        if self == RobotState.CRASH:
            return RobotState.CRASH
        elif self == RobotState.LOOK:
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
import os
print("Current Working Directory:", os.getcwd())
