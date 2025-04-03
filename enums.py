from enum import Enum, auto

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

# ===== NEW ENUMS ADDED BELOW =====
class FaultType(str, Enum):
    NONE = "None"
    CRASH = "Crash"          # Robot stops all actions
    BYZANTINE = "Byzantine"  # Robot sends corrupted data
    DELAY = "Delay"          # Robot has movement delays

class VisibilityState(str, Enum):
    CLEAR = "Clear"          # Normal visibility
    LIMITED = "Limited"      # Restricted by distance
    OBSTRUCTED = "Obstructed" # Blocked by obstacles

class Orientation(str, Enum):
    NORTH = "North"
    SOUTH = "South" 
    EAST = "East"
    WEST = "West"

# (Kept original diagnostic print)
import os
print("Current Working Directory:", os.getcwd())
