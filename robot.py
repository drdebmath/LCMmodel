import math
from typing import NamedTuple, Callable, List, Dict, Tuple, Union # Keep Union for type hints if needed

class RobotState:
    LOOK = "LOOK"
    MOVE = "MOVE"
    WAIT = "WAIT"
    TERMINATED = "TERMINATED"
    CRASH = "CRASH"

    @staticmethod
    def next_state(current_state: str) -> str: # Add type hints here too
        if current_state == RobotState.CRASH:
            return RobotState.CRASH
        elif current_state == RobotState.LOOK:
            return RobotState.MOVE
        elif current_state == RobotState.MOVE:
            return RobotState.WAIT
        elif current_state == RobotState.WAIT:
            return RobotState.LOOK
        return current_state # Default fallback

class SchedulerType:
    ASYNC = "Async"

class DistributionType:
    EXPONENTIAL = "Exponential"

class Algorithm:
    GATHERING = "Gathering"
    SEC = "SEC"


Time = float
Id = int

class Coordinates(NamedTuple):
    x: float
    y: float
    def __str__(self):
        return f"({float(self.x):.4f}, {float(self.y):.4f})"

class Circle(NamedTuple):
    center: Coordinates
    radius: float
    def __str__(self):
        return f"Center: {self.center} ; radius: {float(self.radius):.4f}"

class SnapshotDetails(NamedTuple):
    pos: Coordinates
    state: str
    frozen: bool
    terminated: bool
    multiplicity: Union[int, None] # Use Union for type hint clarity

class Event(NamedTuple):
    time: Time # Use type alias
    id: Id    # Use type alias
    state: str

class Orientation(NamedTuple): # Keep definition even if unused for now
    translation: float
    rotation: float
    reflection: float

# --- Robot Class Definition (Mostly unchanged logic) ---

# Simple logging replacement
class SimpleLogger:
    def info(self, msg):
        print(f"INFO: {msg}")
    def warning(self, msg):
        print(f"WARN: {msg}")
    def error(self, msg):
        print(f"ERROR: {msg}")

class Robot:
    _logger = SimpleLogger()
    _generator = None # Will be set by scheduler

    def __init__(
        self,
        id: Id, # Use type alias
        coordinates: Coordinates,
        algorithm: str, # e.g., Algorithm.GATHERING
        speed: float = 1.0,
        color: Union[str, None] = None, # Use Union
        visibility_radius: Union[float, None] = None, # Use Union
        # orientation: Union[Orientation, None] = None, # Keep if needed later
        # obstructed_visibility: bool = False, # Keep if needed later
        multiplicity_detection: bool = False,
        rigid_movement: bool = False,
        threshold_precision: float = 5,
    ):
        self.speed = speed
        self.color = color
        # Ensure visibility_radius is float or inf
        self.visibility_radius = float(visibility_radius) if visibility_radius is not None else float('inf')
        # self.obstructed_visibility = obstructed_visibility
        self.multiplicity_detection = multiplicity_detection
        self.rigid_movement = rigid_movement
        # self.orientation = orientation
        self.start_time: Union[Time, None] = None # Use type alias and Union
        self.end_time: Union[Time, None] = None   # Use type alias and Union
        self.state: str = RobotState.WAIT # Use constant from this file
        self.start_position: Coordinates = coordinates
        self.calculated_position: Union[Coordinates, None] = None
        self.number_of_activations: int = 0
        self.travelled_distance: float = 0.0
        self.snapshot: Union[Dict[Id, SnapshotDetails], None] = None # Use Dict, Id
        self.coordinates: Coordinates = coordinates
        self.id: Id = id # Use type alias
        self.threshold_precision: float = threshold_precision
        self.frozen: bool = False
        self.terminated: bool = False
        self.sec: Union[Circle, None] = None # Stores the calculated SEC

        # Assign algorithm type based on the input string constant
        if algorithm == Algorithm.GATHERING:
             self.algorithm_type = Algorithm.GATHERING
        elif algorithm == Algorithm.SEC:
             self.algorithm_type = Algorithm.SEC
        else:
             raise ValueError(f"Unknown algorithm: {algorithm}")


    def set_faulty(self, faulty: bool) -> None:
        if faulty:
            self.state = RobotState.CRASH

    def look(
        self,
        snapshot: Dict[Id, SnapshotDetails], # Use Dict, Id
        time: Time,                         # Use Time
    ) -> None:
        if self.state == RobotState.CRASH: return

        self.state = RobotState.LOOK

        self.snapshot = {}
        for key, value in snapshot.items():
            if self.visibility_radius == float('inf') or self._robot_is_visible(value.pos):
                transformed_pos = self._convert_coordinate(value.pos)
                # Make sure SnapshotDetails is used correctly
                self.snapshot[key] = SnapshotDetails(
                    transformed_pos,
                    value.state,
                    value.frozen,
                    value.terminated,
                    value.multiplicity,
                )

        Robot._logger.info(
            f"[{time:.2f}] {{R{self.id}}} LOOK    -- Snapshot {self.prettify_snapshot(self.snapshot)}"
        )

        active_visible_robots = [r for r_id, r in self.snapshot.items() if not r.terminated and r.state != RobotState.CRASH]

        if len(active_visible_robots) <= 1 and self.id in self.snapshot:
            self.frozen = True
            self.terminated = True
            Robot._logger.info(f"[{time:.2f}] {{R{self.id}}} TERMINATED (only self visible)")
            self.wait(time)
            return

        algo, algo_terminal = self._select_algorithm()
        # Type hint for algo: Callable[[], Tuple[Coordinates, List[any]]]
        # Type hint for algo_terminal: Callable[[Coordinates, List[any]], bool]
        self.calculated_position = self._compute(algo, algo_terminal, time)
        pos_str = (
            f"({self.calculated_position.x:.4f}, {self.calculated_position.y:.4f})" # Access .x, .y
            if self.calculated_position
            else "None"
        )
        Robot._logger.info(
            f"[{time:.2f}] {{R{self.id}}} COMPUTE -- Computed Pos: {pos_str}"
        )

        if self.terminated:
             Robot._logger.info(f"[{time:.2f}] {{R{self.id}}} TERMINATED (condition met in compute)")
             self.wait(time)
             return

        if self.calculated_position is None or \
           math.dist(self.calculated_position, self.coordinates) < 10**-self.threshold_precision:
            self.frozen = True
            Robot._logger.info(f"[{time:.2f}] {{R{self.id}}} FROZEN (target reached or no movement)")
            self.wait(time)
        else:
            self.frozen = False

    def _compute(
        self,
        algo: Callable[[], Tuple[Coordinates, List[any]]],
        check_terminal: Callable[[Coordinates, List[any]], bool],
        time: Time
    ) -> Union[Coordinates, None]:
        try:
            coord, extra_args = algo()

            if check_terminal is None:
                Robot._logger.error("Algorithm termination function not passed in")
                return self.coordinates

            if check_terminal(coord, extra_args):
                Robot._logger.info(f"[{time:.2f}] {{R{self.id}}} Termination condition met during compute.")
                self.terminated = True
                return coord # Return calculated coord, but flag is set

            else:
                 return coord

        except Exception as e:
            Robot._logger.error(f"[{time:.2f}] {{R{self.id}}} Error during _compute: {e}")
            self.frozen = True
            return self.coordinates


    def move(self, start_time: Time) -> None:
        if self.state == RobotState.CRASH or self.terminated or self.frozen:
             Robot._logger.info(f"[{start_time:.2f}] {{R{self.id}}} Skipping MOVE (State: {self.state}, Term: {self.terminated}, Frozen: {self.frozen})")
             self.state = RobotState.WAIT # Ensure it goes back to WAIT if frozen/terminated/crashed tried to move
             return

        if self.calculated_position is None:
             Robot._logger.warning(f"[{start_time:.2f}] {{R{self.id}}} MOVE called with no calculated_position. Skipping move.")
             self.state = RobotState.WAIT
             return

        self.state = RobotState.MOVE
        Robot._logger.info(f"[{start_time:.2f}] {{R{self.id}}} MOVE -> {self.calculated_position}")
        self.start_time = start_time
        self.start_position = self.coordinates

    def wait(self, time: Time) -> None:
        final_pos = self.get_position(time)
        current_distance = 0.0
        if self.start_time is not None and self.state == RobotState.MOVE:
             # Make sure start_position is Coordinates type
             current_distance = math.dist(self.start_position, final_pos)
             self.travelled_distance += current_distance

        self.coordinates = final_pos

        self.start_time = None
        self.end_time = time
        self.state = RobotState.WAIT

        Robot._logger.info(
            f"[{time:.2f}] {{R{self.id}}} WAIT    -- Pos: {self.coordinates} Dist: {current_distance:.4f} Total: {self.travelled_distance:.4f} Frozen: {self.frozen} Term: {self.terminated}"
        )


    def get_position(self, time: Time) -> Coordinates:
        if self.state != RobotState.MOVE or self.start_time is None or self.calculated_position is None:
            return self.coordinates

        target_distance = math.dist(self.start_position, self.calculated_position)

        if target_distance < 1e-9:
            return self.calculated_position

        elapsed_time = time - self.start_time
        distance_covered = self.speed * elapsed_time

        if distance_covered >= target_distance - (10**-self.threshold_precision):
            return self.calculated_position
        else:
            factor = distance_covered / target_distance
            interpolated_coords = self._interpolate(
                self.start_position, self.calculated_position, factor
            )
            return interpolated_coords


    def _select_algorithm(self) -> Tuple[Callable, Callable]: # Return types are complex, using Tuple[Callable, Callable]
        if self.algorithm_type == Algorithm.GATHERING:
            return (self._midpoint, self._midpoint_terminal)
        elif self.algorithm_type == Algorithm.SEC:
            return (self._smallest_enclosing_circle, self._sec_terminal)
        else:
            raise ValueError(f"Invalid algorithm type: {self.algorithm_type}")

    def _interpolate(
        self, start: Coordinates, end: Coordinates, t: float
    ) -> Coordinates:
        t = max(0.0, min(1.0, t))
        # Ensure Coordinates constructor is used
        return Coordinates(
            start.x + t * (end.x - start.x), start.y + t * (end.y - start.y)
        )

    def _convert_coordinate(self, coord: Coordinates) -> Coordinates:
        return coord

    def _robot_is_visible(self, coord: Coordinates) -> bool:
        if self.visibility_radius == float('inf'):
             return True
        distance = math.dist(self.coordinates, coord)
        return distance <= self.visibility_radius


    def _midpoint(self) -> Tuple[Coordinates, List[any]]:
        if not self.snapshot:
             Robot._logger.warning(f"{{R{self.id}}} Midpoint calculation with empty snapshot. Staying put.")
             return (self.coordinates, [])

        x = y = 0
        num_visible = 0
        for _, value in self.snapshot.items():
            x += value.pos.x # Access pos.x
            y += value.pos.y # Access pos.y
            num_visible += 1

        if num_visible == 0:
             return (self.coordinates, [])

        x /= num_visible
        y /= num_visible

        return (Coordinates(x, y), []) # Construct Coordinates


    def _midpoint_terminal(self, coord: Coordinates, args: List[any] = None) -> bool:
        if not self.snapshot:
             return True

        for robot_id, details in self.snapshot.items():
            if details.state != RobotState.CRASH:
                 # Access details.pos
                if math.dist(details.pos, coord) > math.pow(10, -self.threshold_precision):
                    return False
        return True

    # --- SEC Algorithm Methods ---
    # Ensure Coordinates, Circle, SnapshotDetails, etc. are used correctly from this file's definitions
    # Make sure .x, .y are accessed correctly on Coordinates objects

    def _smallest_enclosing_circle(self) -> Tuple[Coordinates, List[Union[Circle, None]]]:
        visible_robots_details = {rid: r for rid, r in self.snapshot.items() if r.state != RobotState.CRASH}
        points_coords: List[Coordinates] = [r.pos for r in visible_robots_details.values()] # List of Coordinates

        num_robots = len(points_coords)
        destination: Union[Coordinates, None] = None
        calculated_sec: Union[Circle, None] = None

        try:
            if num_robots == 0:
                destination = self.coordinates
                calculated_sec = None
            elif num_robots == 1:
                destination = points_coords[0]
                calculated_sec = Circle(points_coords[0], 0) # Use Circle constructor
            elif num_robots == 2:
                a, b = points_coords[0], points_coords[1]
                calculated_sec = self._circle_from_two(a, b)
                destination = self._closest_point_on_circle(calculated_sec, self.coordinates)
            elif num_robots == 3:
                potential_sec = None
                for i in range(num_robots):
                    for j in range(i + 1, num_robots):
                        a, b = points_coords[i], points_coords[j]
                        sec_candidate = self._circle_from_two(a, b)
                        if self._valid_circle(sec_candidate, points_coords):
                            potential_sec = sec_candidate
                            break
                    if potential_sec: break

                if not potential_sec:
                     a, b, c = points_coords[0], points_coords[1], points_coords[2]
                     if abs(a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) < 1e-9:
                         max_dist_sq = -1
                         p1, p2 = a, b
                         pairs = [(a, b), (a, c), (b, c)]
                         for p_i, p_j in pairs:
                             d_sq = (p_i.x - p_j.x)**2 + (p_i.y - p_j.y)**2
                             if d_sq > max_dist_sq:
                                 max_dist_sq = d_sq
                                 p1, p2 = p_i, p_j
                         potential_sec = self._circle_from_two(p1, p2)
                     else:
                         potential_sec = self._circle_from_three(a, b, c)

                calculated_sec = potential_sec
                if calculated_sec:
                     destination = self._closest_point_on_circle(calculated_sec, self.coordinates)
                else:
                     Robot._logger.warning(f"[{self.id}] Failed to calculate SEC for 3 points. Staying put.")
                     destination = self.coordinates
            else: # > 3 robots
                calculated_sec = self._sec_welzl_coords(points_coords)
                if calculated_sec:
                    destination = self._closest_point_on_circle(calculated_sec, self.coordinates)
                else:
                     Robot._logger.warning(f"[{self.id}] Failed to calculate SEC via Welzl. Staying put.")
                     destination = self.coordinates

            self.sec = calculated_sec
            # Ensure destination is always Coordinates
            final_destination = destination if destination is not None else self.coordinates
            return (final_destination, [self.sec])

        except Exception as e:
            Robot._logger.error(f"[{self.id}] Error in _smallest_enclosing_circle: {e}")
            return (self.coordinates, [None])


    def _sec_terminal(self, _, args: List[Union[Circle, None]]) -> bool:
        if not args or args[0] is None:
            return False

        circle: Circle = args[0] # Now explicitly Circle type

        visible_robots_details = {rid: r for rid, r in self.snapshot.items() if r.state != RobotState.CRASH}

        if not visible_robots_details:
             return True

        for robot_id, details in visible_robots_details.items():
            # Pass details.pos (Coordinates) to is_point_on_circle
            if not self._is_point_on_circle(details.pos, circle):
                 return False
        return True


    def _sec_welzl_coords(self, points: List[Coordinates]) -> Union[Circle, None]:
        if not points: return None
        points_copy = points.copy()
        if Robot._generator is None:
             Robot._logger.error("Robot._generator not set for Welzl shuffle!")
             import random
             random.shuffle(points_copy)
        else:
            Robot._generator.shuffle(points_copy)
        return self._sec_welzl_recur_coords(points_copy, [], len(points_copy))


    def _sec_welzl_recur_coords(self, P: List[Coordinates], R: List[Coordinates], n: int) -> Circle:
        if n == 0 or len(R) == 3:
            return self._min_circle(R)

        idx = Robot._generator.integers(0, n) if n > 0 else 0
        p = P[idx]
        P[idx], P[n - 1] = P[n - 1], P[idx]

        c = self._sec_welzl_recur_coords(P, R.copy(), n - 1)

        # Use math.dist with Coordinates objects
        if c is not None and c.radius >= 0 and \
           round(math.dist(c.center, p), self.threshold_precision) <= round(c.radius, self.threshold_precision):
             return c
        else:
            R.append(p)
            return self._sec_welzl_recur_coords(P, R.copy(), n - 1)


    def _min_circle(self, points: List[Coordinates]) -> Circle:
        if not points:
            return Circle(Coordinates(0, 0), 0) # Use constructors
        elif len(points) == 1:
            return Circle(points[0], 0)
        elif len(points) == 2:
            return self._circle_from_two(points[0], points[1])
        elif len(points) == 3:
             for i in range(3):
                 p1, p2 = points[i], points[(i + 1) % 3]
                 c = self._circle_from_two(p1, p2)
                 p3 = points[(i + 2) % 3]
                 # Use math.dist with Coordinates
                 if round(math.dist(c.center, p3), self.threshold_precision) <= round(c.radius, self.threshold_precision):
                      return c

             a, b, c_pts = points[0], points[1], points[2] # Rename c to avoid conflict
             # Access .x, .y for collinearity check
             if abs(a.x * (b.y - c_pts.y) + b.x * (c_pts.y - a.y) + c_pts.x * (a.y - b.y)) < 1e-9:
                 max_dist_sq = -1
                 p1_max, p2_max = a, b
                 pairs = [(a, b), (a, c_pts), (b, c_pts)]
                 for p_i, p_j in pairs:
                     # Access .x, .y for distance check
                     d_sq = (p_i.x - p_j.x)**2 + (p_i.y - p_j.y)**2
                     if d_sq > max_dist_sq:
                         max_dist_sq = d_sq
                         p1_max, p2_max = p_i, p_j
                 return self._circle_from_two(p1_max, p2_max)
             else:
                 return self._circle_from_three(a, b, c_pts)
        else:
             Robot._logger.error("Min_circle called with > 3 points")
             return Circle(Coordinates(0,0), -1)


    def _is_point_on_circle(self, p: Coordinates, c: Circle) -> bool:
        if c is None or c.radius < 0: return False
        # Use math.dist with Coordinates objects
        distance = math.dist(p, c.center)
        return abs(distance - c.radius) < math.pow(10, -self.threshold_precision)


    def _closest_point_on_circle(self, circle: Circle, point: Coordinates) -> Coordinates:
        if circle is None or circle.radius < 0: return point

        center: Coordinates = circle.center
        radius: float = circle.radius

        # Use math.dist with Coordinates
        if math.dist(center, point) < 1e-9:
             return Coordinates(center.x + radius, center.y) # Use constructor

        # Access .x, .y
        vx, vy = point.x - center.x, point.y - center.y
        d = math.sqrt(vx**2 + vy**2)
        scale = radius / d
        cx = center.x + vx * scale
        cy = center.y + vy * scale
        return Coordinates(cx, cy) # Use constructor


    def _valid_circle(self, circle: Circle, points: List[Coordinates]) -> bool:
        if circle is None or circle.radius < 0: return False
        for p in points:
            # Use math.dist with Coordinates
            if round(math.dist(circle.center, p), self.threshold_precision) > round(circle.radius, self.threshold_precision):
                return False
        return True


    def _circle_from_two(self, a: Coordinates, b: Coordinates) -> Circle:
        # Access .x, .y
        center_x = (a.x + b.x) / 2.0
        center_y = (a.y + b.y) / 2.0
        center = Coordinates(center_x, center_y) # Use constructor
        # Use math.dist with Coordinates
        radius = math.dist(a, b) / 2.0
        return Circle(center, radius) # Use constructor


    def _circle_from_three(self, a: Coordinates, b: Coordinates, c: Coordinates) -> Circle:
        # Access .x, .y
        A = b.x - a.x; B = b.y - a.y
        C = c.x - a.x; D = c.y - a.y
        E = A * (a.x + b.x) + B * (a.y + b.y)
        F = C * (a.x + c.x) + D * (a.y + c.y)
        G = 2 * (A * (c.y - b.y) - B * (c.x - b.x))

        if abs(G) < 1e-9:
             Robot._logger.warning(f"[{self.id}] _circle_from_three called with collinear points: {a}, {b}, {c}. Using diameter fallback.")
             max_dist_sq = -1
             p1_max, p2_max = a, b
             pairs = [(a, b), (a, c), (b, c)]
             for p_i, p_j in pairs:
                 d_sq = (p_i.x - p_j.x)**2 + (p_i.y - p_j.y)**2
                 if d_sq > max_dist_sq:
                     max_dist_sq = d_sq
                     p1_max, p2_max = p_i, p_j
             return self._circle_from_two(p1_max, p2_max)

        center_x = (D * E - B * F) / G
        center_y = (A * F - C * E) / G
        center = Coordinates(center_x, center_y) # Use constructor
        # Use math.dist with Coordinates
        radius = math.dist(center, a)
        return Circle(center, radius) # Use constructor

    # --- End SEC ---

    def prettify_snapshot(self, snapshot: Dict[Id, SnapshotDetails]) -> str:
        if not snapshot: return " <empty>"
        result = ""
        sorted_ids = sorted(snapshot.keys())
        for key in sorted_ids:
            value = snapshot[key] # value is SnapshotDetails
            frozen = "*" if value.frozen else ""
            terminated = "#" if value.terminated else ""
            crashed = "!" if value.state == RobotState.CRASH else ""
            multi = f"({value.multiplicity})" if self.multiplicity_detection and value.multiplicity and value.multiplicity > 1 else ""
            state_str = value.state
            # value.pos is Coordinates
            result += f"\n\t{key}{frozen}{terminated}{crashed}{multi}: {state_str} @ {value.pos}"
        return result


    def __str__(self):
         state_str = self.state
         term_str = "#" if self.terminated else ""
         frozen_str = "*" if self.frozen else ""
         crash_str = "!" if state_str == RobotState.CRASH else ""
         # self.coordinates is Coordinates
         return f"R{self.id}{term_str}{frozen_str}{crash_str} @ {self.coordinates}, St: {state_str}, Spd: {self.speed:.2f}, VRad: {self.visibility_radius}"

print("robot.py including types/enums loaded.")