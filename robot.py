from enums import RobotState
from type_defs import *
import math


class Robot:
    def __init__(
        self,
        id: int,
        speed: float = 1.0,
        color: str | None = None,
        visibility_radius: float | None = None,
        orientation: tuple[float, float, float] | None = None,
        obstructed_visibility: bool = False,
        multiplicity_detection: bool = False,
        rigid_movement: bool = False,
        coordinates: Coordinates = None,
    ):
        self.speed = speed
        self.color = color
        self.visibility_radius = visibility_radius
        self.obstructed_visibility = obstructed_visibility
        self.multiplicity_detection = multiplicity_detection
        self.rigid_movement = rigid_movement
        self.orientation = orientation
        self.start_time = None
        self.end_time = None
        self.current_time = None
        self.state = RobotState.WAIT
        self.start_position = coordinates
        self.calculated_position = None
        self.number_of_activations = 0
        self.travelled_distance = 0.0
        self.snapshot = None
        self.coordinates = coordinates
        self.id = id

    def look(self, snapshot: dict[Id, tuple[Coordinates, State]], time: float) -> None:
        self.snapshot = {
            key: self.convert_coordinate(value)
            for key, value in snapshot.items()
            if self.robot_is_visible(value[0])
        }
        print(f"[{time:.6f}] {{R{self.id}}} LOOK    -- Snapshot {self.snapshot}")

        self.calculated_position = self.compute(self.midpoint)
        pos_str = f"({float(self.calculated_position[0]):.6f}, {float(self.calculated_position[1]):.6f})"
        print(f"[{time:.6f}] {{R{self.id}}} COMPUTE -- Computed Pos: {pos_str}")

    def compute(self, algo) -> Coordinates:
        coord = algo()
        return coord

    def move(self, start_time: float) -> None:
        print(f"[{start_time:.6f}] {{R{self.id}}} MOVE")

        self.start_time = start_time

    def wait(self, time: float) -> None:
        self.end_time = time

        self.coordinates = self.get_position(time)
        self.travelled_distance += math.dist(self.start_position, self.coordinates)

        self.start_position = self.coordinates
        print(
            f"[{time:.6f}] {{R{self.id}}} WAIT    -- Travelled a total of {self.travelled_distance} units"
        )

        self.calculated_position = None
        self.start_time = None
        self.end_time = None

    def get_position(self, time: float) -> Coordinates:
        self.current_time = time

        if self.calculated_position == None:
            return self.coordinates

        distance = math.dist(self.start_position, self.calculated_position)
        distance_covered = self.speed * time

        if distance_covered > distance:
            self.coordinates = self.calculated_position
        else:
            factor = distance_covered / distance
            self.coordinates = self.interpolate(
                self.start_position, self.calculated_position, factor
            )

        return self.coordinates

    def interpolate(
        self, start: Coordinates, end: Coordinates, t: float
    ) -> Coordinates:
        return (start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]))

    def convert_coordinate(self, coord: Coordinates) -> Coordinates:
        return coord

    def robot_is_visible(self, coord: Coordinates):
        return True

    def midpoint(self) -> Coordinates:
        x = y = 0
        for _, value in self.snapshot.items():
            x += value[0][0]
            y += value[0][1]

        x = x / len(self.snapshot)
        y = y / len(self.snapshot)

        return (x, y)

    def center(self) -> Coordinates:
        return (0, 0)

    def __str__(self):
        return f"R{self.id}, speed: {self.speed}, color: {self.color}, coordinates: {self.coordinates}"
