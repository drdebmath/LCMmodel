from typing import NamedTuple

Time = float
Id = int

class Coordinates(NamedTuple):
    x: float
    y: float
    def __str__(self):
        return f"({float(self.x)}, {float(self.y)})"

class Circle(NamedTuple):
    center: Coordinates
    radius: float
    def __str__(self):
        return f"Center: {self.center} ; radius: {float(self.radius)}"

class SnapshotDetails(NamedTuple):
    pos: Coordinates
    state: str
    frozen: bool
    terminated: bool
    multiplicity: int | None
    light: str | None   # New field for the robot's current light

class Event(NamedTuple):
    time: float
    id: int
    state: str

class Orientation(NamedTuple):
    translation = float
    rotation = float
    reflection = float
