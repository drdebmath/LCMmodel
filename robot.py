from enums import RobotState, Algorithm
from type_defs import *
from typing import Callable
import math
import logging
import numpy as np


class Robot:
    _logger: logging.Logger | None = None
    _generator = None

    def __init__(
        self,
        logger: logging.Logger,
        id: int,
        coordinates: Coordinates,
        algorithm: str,
        speed: float = 1.0,
        color: str | None = None,
        visibility_radius: float | None = None,
        orientation: Orientation | None = None,
        obstructed_visibility: bool = False,
        multiplicity_detection: bool = False,
        rigid_movement: bool = False,
        threshold_precision: float = 5,
        fault_type: str | None = None,
    ):
        Robot._logger = logger
        self.speed = speed
        self.color = color
        self.visibility_radius = visibility_radius
        self.obstructed_visibility = obstructed_visibility
        self.multiplicity_detection = multiplicity_detection
        self.rigid_movement = rigid_movement
        self.orientation = orientation
        self.start_time = None
        self.end_time = None
        self.state = RobotState.WAIT
        self.start_position = coordinates
        self.calculated_position = None
        self.number_of_activations = 0
        self.travelled_distance = 0.0
        self.snapshot: dict[Id, SnapshotDetails] | None = None
        self.coordinates = coordinates
        self.id = id
        self.threshold_precision = threshold_precision
        self.frozen = False  # true if we skipped move step
        self.terminated = False
        self.sec = None  # Stores the calculated SEC
        self.fault_type = fault_type  # store fault type

        match algorithm:
            case "Gathering":
                self.algorithm = Algorithm.GATHERING
            case "SEC":
                self.algorithm = Algorithm.SEC

    def look(self, snapshot: dict[Id, SnapshotDetails], time: float) -> None:
        self.state = RobotState.LOOK

        # Byzantine Fault: Modify snapshot before processing
        if self.fault_type == "byzantine":
            snapshot = self._introduce_byzantine_error(snapshot)
            Robot._logger.info(f"R{self.id} is Byzantine and modified its snapshot.")

        self.snapshot = {}
        for key, value in snapshot.items():
            if self._robot_is_visible(value.pos):
                transformed_pos = self._convert_coordinate(value.pos)
                self.snapshot[key] = SnapshotDetails(
                    transformed_pos,
                    value.state,
                    value.frozen,
                    value.terminated,
                    value.multiplicity,
                )

        Robot._logger.info(
            f"[{time}] {{R{self.id}}} LOOK    -- Snapshot {self.prettify_snapshot(snapshot)}"
        )

        if len(self.snapshot) == 1:
            self.frozen = True
            self.terminated = True
            self.wait(time)
            return

        algo, algo_terminal = self._select_algorithm()
        self.calculated_position = self._compute(algo, algo_terminal)
        pos_str = (
            f"({self.calculated_position[0]}, {self.calculated_position[1]})"
            if self.calculated_position
            else ""
        )
        Robot._logger.info(
            f"[{time}] {{R{self.id}}} COMPUTE -- Computed Pos: {pos_str}"
        )

        if (
            math.dist(self.calculated_position, self.coordinates)
            < 10**-self.threshold_precision
        ):
            self.frozen = True
            self.wait(time)
        else:
            self.frozen = False

    def _compute(
        self,
        algo: Callable[[], tuple[Coordinates, list[any]]],
        check_terminal: Callable[[Coordinates, list[any]], bool],
    ) -> Coordinates:
        # extra args that check_terminal might need
        coord, extra_args = algo()

        if check_terminal == None:
            raise Exception("Algorithm termination function not passed in")

        if check_terminal(coord, extra_args) == True:
            self.terminated = True

        return coord

    def move(self, start_time: float) -> None:
        self.state = RobotState.MOVE
        Robot._logger.info(f"[{start_time}] {{R{self.id}}} MOVE")

        self.start_time = start_time
        self.start_position = self.coordinates

    def wait(self, time: float) -> None:
        self.coordinates = self.get_position(time)
        self.end_time = time
        self.state = RobotState.WAIT

        current_distance = math.dist(self.start_position, self.coordinates)
        self.travelled_distance += current_distance

        Robot._logger.info(
            f"[{time}] {{R{self.id}}} WAIT    -- Distance: {current_distance} | Total Distance: {self.travelled_distance} units"
        )

        self.start_time = None
        self.end_time = None

    def get_position(self, time: float) -> Coordinates:
        if self.state == RobotState.LOOK or self.state == RobotState.WAIT:
            return self.coordinates

        distance = math.dist(self.start_position, self.calculated_position)
        distance_covered = self.speed * (time - self.start_time)

        if (
            distance_covered > distance
            or abs(distance_covered - distance) < 10**-self.threshold_precision
        ):
            self.coordinates = self.calculated_position
        else:
            factor = distance_covered / distance
            self.coordinates = self._interpolate(
                self.start_position, self.calculated_position, factor
            )

        return self.coordinates

    def _select_algorithm(self):
        match self.algorithm:
            case Algorithm.GATHERING:
                return (self._midpoint, self._midpoint_terminal)
            case Algorithm.SEC:
                return (self._smallest_enclosing_circle, self._sec_terminal)

    def _interpolate(
        self, start: Coordinates, end: Coordinates, t: float
    ) -> Coordinates:
        return Coordinates(
            start.x + t * (end.x - start.x), start.y + t * (end.y - start.y)
        )

    def _convert_coordinate(self, coord: Coordinates) -> Coordinates:
        return coord

    def _robot_is_visible(self, coord: Coordinates):
        distance = math.dist(self.coordinates, coord)

        return self.visibility_radius > distance

    def _midpoint(self) -> tuple[Coordinates, list[any]]:
        x = y = 0
        for _, value in self.snapshot.items():
            x += value.pos.x
            y += value.pos.y

        x = x / len(self.snapshot)
        y = y / len(self.snapshot)

        return (Coordinates(x, y), [])

    def _midpoint_terminal(self, coord: Coordinates, args=None) -> bool:
        robot_ids = self.snapshot.keys()
        for id in robot_ids:
            if math.dist(self.snapshot[id].pos, coord) > math.pow(
                10, -self.threshold_precision
            ):
                return False
        return True

    def _smallest_enclosing_circle(self) -> tuple[Coordinates, list[Circle]]:
        """Intermediary function that calls sec algorithms based on num of robots"""
        ids = self._get_visible_robots()
        num_robots = len(self.snapshot)
        destination: Coordinates | None = None
        if num_robots == 0:
            destination = (0, 0)
        elif num_robots == 1:
            destination = self.snapshot[0].pos
        elif num_robots == 2:
            i, j = ids[0], ids[1]
            a, b = self.snapshot[i].pos, self.snapshot[j].pos
            self.sec = self._circle_from_two(a, b)
            destination = self._closest_point_on_circle(self.sec, self.coordinates)
        elif num_robots == 3:
            # find sec using 2 points, otherwise find sec intersecting 3 points
            for i in range(num_robots):
                for j in range(i + 1, num_robots):
                    a, b = self.snapshot[ids[i]].pos, self.snapshot[ids[j]].pos
                    sec = self._circle_from_two(a, b)
                    if self._valid_circle(sec):
                        self.sec = sec
            if not self.sec:
                i, j, k = ids[0], ids[1], ids[2]
                a, b, c = (
                    self.snapshot[i].pos,
                    self.snapshot[j].pos,
                    self.snapshot[k].pos,
                )
                self.sec = self._circle_from_three(a, b, c)
            destination = self._closest_point_on_circle(self.sec, self.coordinates)
        else:
            # self.sec = self._sec()
            self.sec = self._sec_welzl(ids)
            destination = self._closest_point_on_circle(self.sec, self.coordinates)
        return (destination, [self.sec])

    def _sec_terminal(self, _, args: list[Circle]) -> bool:
        """Determines terminal state for robot"""
        ids = self._get_visible_robots()

        circle = args[0]

        if circle == None:
            return True

        for i in ids:
            if not self._is_point_on_circle(self.snapshot[i].pos, circle):
                return False
        return True

    def _sec_welzl(self, points: list[Id]) -> Circle:
        """
        Returns smallest enclosing circle given number of robots in the form of
        (Center, Radius)
        Time Complexity: O(n)
        """
        points_copy = points.copy()
        Robot._generator.shuffle(points_copy)
        return self._sec_welzl_recur(points_copy, [], len(points_copy))

    def _sec_welzl_recur(
        self, points: list[Id], R: list[Coordinates], n: int
    ) -> Circle:
        if n == 0 or len(R) == 3:
            return self._min_circle(R)
        idx = Robot._generator.integers(0, n - 1) if n > 1 else 0
        p = self.snapshot[points[idx]].pos
        points[idx], points[n - 1] = points[n - 1], points[idx]
        c = self._sec_welzl_recur(points, R.copy(), n - 1)
        if round(math.dist(c.center, p), self.threshold_precision) <= c.radius:
            return c
        R.append(p)
        return self._sec_welzl_recur(points, R.copy(), n - 1)

    def _min_circle(self, points: list[Coordinates]) -> Circle:
        if not points:
            return Circle(Coordinates(0, 0), 0)
        elif len(points) == 1:
            return Circle(points[0], 0)
        elif len(points) == 2:
            return self._circle_from_two(points[0], points[1])
        for i in range(3):
            for j in range(i + 1, 3):
                c = self._circle_from_two(points[i], points[j])
                if self._valid_circle(c, points):
                    return c
        return self._circle_from_three(points[0], points[1], points[2])

    def _is_acute_triangle(
        self, a: Coordinates, b: Coordinates, c: Coordinates
    ) -> bool:
        # Calculate squared lengths of each side
        ab_sq = (a.x - b.x) ** 2 + (a.y - b.y) ** 2
        bc_sq = (b.x - c.x) ** 2 + (b.y - c.y) ** 2
        ca_sq = (c.x - a.x) ** 2 + (c.y - a.y) ** 2

        # Check for the acute triangle condition
        return (
            (ab_sq + bc_sq > ca_sq)
            and (ab_sq + ca_sq > bc_sq)
            and (bc_sq + ca_sq > ab_sq)
        )

    def _is_point_on_circle(self, p: Coordinates, c: Circle) -> bool:
        distance = math.sqrt((p.x - c.center.x) ** 2 + (p.y - c.center.y) ** 2)

        return abs(distance - c.radius) < math.pow(10, -self.threshold_precision)

    def _closest_point_on_circle(
        self, circle: Circle, point: Coordinates
    ) -> Coordinates:
        # Vector from the center of the circle to the point
        center: Coordinates = circle.center
        radius: float = circle.radius
        vx, vy = point.x - center.x, point.y - center.y

        # Distance from the center to the point
        d = math.dist(center, point)

        # Scaling factor to project the point onto the circle
        scale = radius / d

        # Closest point on the circle
        cx = center.x + vx * scale
        cy = center.y + vy * scale

        return Coordinates(cx, cy)

    def _valid_circle(self, circle: Circle, points: list[Coordinates] = None) -> bool:
        """Returns False if at least one point does not lie within given circle"""
        robots = points if points else self.snapshot.items()

        # Iterate through all coordinates
        for robot in robots:
            if not points:
                robot = robot[1].pos
            # If point does not lie inside of the given circle; i.e.: if
            # distance between the center coord and point is more than radius
            if (
                round(math.dist(circle.center, robot), self.threshold_precision)
                > circle.radius
            ):
                return False
        return True

    def _circle_from_two(self, a: Coordinates, b: Coordinates) -> Circle:
        """Returns circle intersecting two points"""

        # Midpoint between a and b
        center = Coordinates((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)
        return Circle(center, math.dist(a, b) / 2.0)

    def _circle_from_three(
        self, a: Coordinates, b: Coordinates, c: Coordinates
    ) -> Circle:
        """Returns circle intersecting three points"""

        # Calculate the midpoints of lines AB and AC
        D = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y))

        if D == 0:
            raise ValueError(
                "Points are collinear; no unique circle can pass through all three points."
            )

        # Calculate circle center coordinates
        ux = (
            (a.x**2 + a.y**2) * (b.y - c.y)
            + (b.x**2 + b.y**2) * (c.y - a.y)
            + (c.x**2 + c.y**2) * (a.y - b.y)
        ) / D
        uy = (
            (a.x**2 + a.y**2) * (c.x - b.x)
            + (b.x**2 + b.y**2) * (a.x - c.x)
            + (c.x**2 + c.y**2) * (b.x - a.x)
        ) / D
        center = Coordinates(ux, uy)

        # Calculate the radius as the distance from the center to any of the three points
        radius = math.sqrt((center.x - a.x) ** 2 + (center.y - a.y) ** 2)

        return Circle(center, radius)

    def _circle_center(self, bx: float, by: float, cx: float, cy: float) -> Coordinates:
        b = bx * bx + by * by
        c = cx * cx + cy * cy
        d = bx * cy - by * cx
        if d == 0:
            return Coordinates(0, 0)
        return Coordinates((cy * b - by * c) / (2 * d), (bx * c - cx * b) / (2 * d))

    def prettify_snapshot(self, snapshot: dict[Id, SnapshotDetails]) -> str:
        result = ""
        for key, value in snapshot.items():
            frozen = "*" if value.frozen == True else ""
            terminated = "#" if value.terminated == True else ""
            result += f"\n\t{key}{frozen}{terminated}: {value[1]} - ({float(value.pos.x),float(value.pos.y)})"

        return result

    def _get_visible_robots(self):
        ids = list(self.snapshot.keys())
        ids.sort()

        return ids

    def __str__(self):
        return f"R{self.id}, speed: {self.speed}, color: {self.color}, coordinates: {self.coordinates}"

    def _introduce_byzantine_error(self, snapshot):
        """Byzantine robots modify snapshot data to mislead others."""
        for key in snapshot.keys():
            if np.random.random() < 0.5:  # 50% chance to corrupt data
                snapshot[key].pos = Coordinates(
                    np.random.uniform(-100, 100),
                    np.random.uniform(-100, 100)
                )
        return snapshot
