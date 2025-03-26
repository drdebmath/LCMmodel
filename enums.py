from enum import Enum, auto
from typing import Dict, List, TypeVar

# Type variable for enum subclassing
E = TypeVar('E', bound='SwarmEnum')

class SwarmEnum(Enum):
    """
    Enhanced base enum class with:
    - Serialization support
    - List generation
    - Type safety
    """
    
    @classmethod
    def names(cls: type[E]) -> List[str]:
        """Get all enum member names as strings"""
        return [e.name for e in cls]
        
    @classmethod 
    def values(cls: type[E]) -> List[E]:
        """Get all enum values"""
        return [e.value for e in cls]

    def __str__(self) -> str:
        """Human-readable format (override per enum if needed)"""
        return self.name.replace('_', ' ').title()

# --- Core Enums (Your Original Implementation) ---
class RobotState(SwarmEnum, str):
    """Robot FSM states with transitions (preserving your exact logic)"""
    LOOK = "LOOK"
    MOVE = "MOVE"
    WAIT = "WAIT"
    TERMINATED = "TERMINATED"

    def next_state(self) -> 'RobotState':
        """Your original state transition logic"""
        if self == RobotState.LOOK:
            return RobotState.MOVE
        elif self == RobotState.MOVE:
            return RobotState.WAIT
        elif self == RobotState.WAIT:
            return RobotState.LOOK
        return RobotState.TERMINATED

class SchedulerType(SwarmEnum):
    """Scheduler types (extended with sync variants)"""
    ASYNC = "Async"
    SYNC = "Sync"
    SEMI_SYNC = "Semi-Sync"

class DistributionType(SwarmEnum):
    """Timing distributions (extended)"""
    EXPONENTIAL = "Exponential"
    UNIFORM = "Uniform"
    NORMAL = "Normal"

# --- Enhanced Enums ---
class Algorithm(SwarmEnum):
    """Algorithms with metadata (preserving your GATHERING/SEC)"""
    GATHERING = auto()
    SEC = auto()
    PATTERN_FORMATION = auto()
    FLOCKING = auto()

    @property
    def description(self) -> str:
        """For UI tooltips"""
        return {
            Algorithm.GATHERING: "Point convergence algorithm",
            Algorithm.SEC: "Secure environment coverage",
        }.get(self, self.name.replace('_', ' '))

class FaultType(SwarmEnum):
    """Fault types with visualization support"""
    CRASH = auto()
    BYZANTINE = auto()
    DELAY = auto()
    OMISSION = auto()

    @property
    def color(self) -> str:
        """Hex colors for visualization"""
        return {
            FaultType.CRASH: '#FF0000',  # Red
            FaultType.BYZANTINE: '#FFA500',  # Orange
            FaultType.DELAY: '#FFFF00',  # Yellow
            FaultType.OMISSION: '#0000FF',  # Blue
        }.get(self, '#808080')  # Gray fallback

# --- Utility Functions ---
def enum_to_dict(enum_cls: type[E]) -> Dict[str, str]:
    """Convert enum to {name: description} for frontend dropdowns"""
    return {e.name: str(e) for e in enum_cls}

def get_fault_color(fault: FaultType) -> str:
    """Shortcut for visualization"""
    return fault.color
