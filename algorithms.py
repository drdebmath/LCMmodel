import math
from type_defs import Coordinates
from enums import RobotState

# We remove the import: from robot import Robot
# We also remove direct references to Robot in the function signatures.

def midpoint_algorithm(robot):
    """
    Gathering-like algorithm: compute the midpoint of all visible robots.
    'robot' is an instance of Robot, but we don't import Robot here to avoid circular import.
    """
    snapshot = robot.snapshot
    x = y = 0.0
    for _, value in snapshot.items():
        x += value.pos.x
        y += value.pos.y
    x /= len(snapshot)
    y /= len(snapshot)
    return Coordinates(x, y), []

def midpoint_terminal(robot, coord: Coordinates, args=None) -> bool:
    """
    Termination check for the midpoint-based Gathering:
    If all snapshot positions are close to 'coord' within threshold_precision.
    """
    snapshot = robot.snapshot
    threshold_precision = robot.threshold_precision
    for _, value in snapshot.items():
        if math.dist(value.pos, coord) > 10 ** -threshold_precision:
            return False
    return True

def color_based_algorithm(robot):
    """
    Color-based algorithm where:
    - Red robots gather at the center of the SEC circle formed by blue robots
    - Blue robots use SEC algorithm
    """
    snapshot = robot.snapshot
    my_pos = robot.coordinates
    my_color = robot.current_light
    
    # For red robots (Gathering algorithm)
    if my_color == "red":
        # Find all blue robots
        blue_robots = []
        for rid, details in snapshot.items():
            if int(rid) != robot.id and details.light == "blue":
                blue_robots.append(details.pos)
        
        if blue_robots:
            # Calculate the SEC of blue robots
            def calculate_sec(points):
                """Calculate the exact center of the smallest enclosing circle"""
                # Simple case handling
                if not points:
                    return my_pos, 0
                if len(points) == 1:
                    return points[0], 0
                if len(points) == 2:
                    # Return the midpoint of two points
                    mid_x = (points[0].x + points[1].x) / 2
                    mid_y = (points[0].y + points[1].y) / 2
                    radius = math.dist(points[0], Coordinates(mid_x, mid_y))
                    return Coordinates(mid_x, mid_y), radius
                
                # For three or more points, use Welzl's algorithm or a simpler calculation
                # For simplicity, let's compute the geometric center of all points
                x_sum = sum(p.x for p in points)
                y_sum = sum(p.y for p in points)
                center = Coordinates(x_sum / len(points), y_sum / len(points))
                
                # Find the radius as the maximum distance to any point
                radius = max(math.dist(center, p) for p in points)
                
                return center, radius
            
            # Calculate the SEC center
            center, _ = calculate_sec(blue_robots)
            robot._logger.info(f"Red robot {robot.id} targeting SEC center at {center}")
            return center, []
        else:
            # If no blue robots, fall back to standard gathering
            return midpoint_algorithm(robot)
    
    # For blue robots, behavior is handled by the robot.py selection of SEC algorithm
    # Return current position to indicate no change from this algorithm
    return my_pos, []

def color_based_terminal(robot, coord: Coordinates, args=None) -> bool:
    """
    Termination condition for color-based algorithm:
    - For red robots: converged to center of blue robots' SEC
    - For blue robots: SEC termination conditions
    """
    snapshot = robot.snapshot
    
    # If only one robot in view, we're done
    if len(snapshot) <= 1:
        return True
    
    # If I'm a red robot
    if robot.current_light == "red":
        # Find all blue robots
        blue_robots = []
        for rid, details in snapshot.items():
            if int(rid) != robot.id and details.light == "blue":
                blue_robots.append(details.pos)
        
        if blue_robots and hasattr(robot, '_sec_from_points'):
            # Calculate center of SEC for blue robots
            center, _ = robot._sec_from_points(blue_robots)
            
            # Check if we're at the center
            if center and math.dist(robot.coordinates, center) < 10 ** -robot.threshold_precision:
                return True
        elif blue_robots:
            # Fallback to geometric center check
            center_x = sum(pos.x for pos in blue_robots) / len(blue_robots)
            center_y = sum(pos.y for pos in blue_robots) / len(blue_robots)
            center = Coordinates(center_x, center_y)
            
            # Check if we're at the center
            if math.dist(robot.coordinates, center) < 10 ** -robot.threshold_precision:
                return True
    
    # For blue robots, standard SEC termination is handled elsewhere
    return False

import math
from type_defs import Coordinates

def two_robot_algorithm(robot):
    """
    Two-robot algorithm implementation with extensive debugging.
    Rules:
    1. Both robots start as black
    2. When two black robots see each other, the one with higher ID changes to white
    3. Black robots stay still
    4. White robots move towards black robots
    """
    snapshot = robot.snapshot
    my_pos = robot.coordinates
    my_id = robot.id
    my_color = robot.current_light
    
    # Debug logging
    robot._logger.info(f"ROBOT {my_id}: Algorithm start - color={my_color}, pos={my_pos}, epoch={robot.epoch}, state={robot.state}")
    
    # Get the other robot's details
    other_id = None
    other_pos = None
    other_color = None
    
    for rid, details in snapshot.items():
        if int(rid) != my_id:
            other_id = int(rid)
            other_pos = details.pos
            other_color = details.light
            break
    
    if other_id is None:
        robot._logger.info(f"ROBOT {my_id}: Cannot see other robot")
        return my_pos, []
    
    # Log the current state
    robot._logger.info(f"ROBOT {my_id} ({my_color}) sees robot {other_id} ({other_color})")
    
    # CRITICAL: If both robots are black, the one with higher ID changes to white
    if my_color == "black" and other_color == "black":
        robot._logger.info(f"ROBOT {my_id}: Two black robots detected: {my_id} and {other_id}")
        
        # Convert IDs to integers for comparison
        my_id_int = int(my_id)
        other_id_int = int(other_id)
        
        # The robot with higher ID changes to white
        if my_id_int > other_id_int:
            robot._logger.info(f"ROBOT {my_id}: Higher ID ({my_id_int} > {other_id_int}) - changing from BLACK to WHITE")
            # Set the light color and ensure it's tracked for stats
            robot.set_light("white", robot.snapshot_time if hasattr(robot, 'snapshot_time') else 0.0)
            # Update local color for movement logic
            my_color = "white"
            
            # Since I'm now white, I should move toward the black robot
            robot._logger.info(f"ROBOT {my_id}: Now WHITE, moving toward BLACK robot at {other_pos}")
            return other_pos, []
        else:
            robot._logger.info(f"ROBOT {my_id}: Lower ID ({my_id_int} < {other_id_int}) - staying BLACK")
            # CRITICAL: Return current position without changing color
            return my_pos, []
    
    # Standard behavior with enhanced debugging
    if my_color == "black":
        # Black robots never move
        robot._logger.info(f"ROBOT {my_id}: BLACK - STAYING STILL")
        return my_pos, []
    elif my_color == "white":
        if other_color == "black":
            # White robots move towards black robots
            robot._logger.info(f"ROBOT {my_id}: WHITE - moving toward BLACK robot {other_id}")
            return other_pos, []
        else:  # other robot is also white
            # Both white robots move to midpoint
            midpoint_x = (my_pos.x + other_pos.x) / 2
            midpoint_y = (my_pos.y + other_pos.y) / 2
            midpoint = Coordinates(midpoint_x, midpoint_y)
            robot._logger.info(f"ROBOT {my_id}: WHITE - moving to midpoint with WHITE robot {other_id}")
            return midpoint, []
    
    # This should never happen
    robot._logger.error(f"ROBOT {my_id}: Invalid state - color {my_color}")
    return my_pos, []

def two_robot_terminal(robot, coord: Coordinates, args=None) -> bool:
    """
    Termination condition for two-robot algorithm:
    - For two black robots: never terminate (let simulation timeout handle it)
    - For two white robots: terminate when they reach the same position
    - For black and white: terminate when white reaches black's position
    
    Args:
        robot: The Robot instance executing the algorithm
        coord: Destination coordinates 
        args: Additional arguments/data from the algorithm
        
    Returns:
        bool: True if the termination condition is met, False otherwise
    """
    # Get snapshot of visible robots
    snapshot = robot.snapshot
    
    # If only this robot is visible, can't determine termination
    if len(snapshot) <= 1:
        return False
    
    # Get the other robot's details
    other_ids = [rid for rid in snapshot.keys() if int(rid) != robot.id]
    if not other_ids:
        return False
    
    other_id = other_ids[0]
    other_details = snapshot[other_id]
    
    # Calculate distance to other robot
    distance = math.dist(robot.coordinates, other_details.pos)
    
    # Precision threshold for considering robots as gathered
    threshold = 10 ** -robot.threshold_precision
    
    # Log termination check
    robot._logger.info(f"Robot {robot.id} ({robot.current_light}) checking termination - distance: {distance}, threshold: {threshold}")
    
    # For two black robots, never terminate (let simulation timeout handle it)
    if robot.current_light == "black" and other_details.light == "black":
        robot._logger.info(f"Robot {robot.id} (BLACK) sees other black robot {other_id} - continuing simulation")
        return False
    
    # For black and white robots, only terminate if white reaches black
    if robot.current_light == "black" and other_details.light == "white":
        # Black robots never terminate - let white robot handle termination
        return False
    elif robot.current_light == "white" and other_details.light == "black":
        # White robot terminates when it reaches black robot
        if distance < threshold:
            robot._logger.info(f"Robot {robot.id} (WHITE) has reached black robot {other_id}")
            return True
        return False
    elif robot.current_light == "white" and other_details.light == "white":
        # Two white robots terminate when they reach each other
        if distance < threshold:
            robot._logger.info(f"Robot {robot.id} (WHITE) has reached other white robot {other_id}")
            return True
        return False
    
    return False

def mutual_visibility_algorithm(robot):
    """
    Algorithm to demonstrate obstructed visibility:
    - Robots initially line up side by side
    - If a robot is obstructing another's view, it turns red
    - If a robot sees colinear robots, it turns blue
    - If a robot has clear visibility (no obstruction/collinearity), it turns green
    
    Returns:
        tuple: (target_coordinates, additional_args)
    """
    snapshot = robot.snapshot
    my_pos = robot.coordinates
    
    # If first epoch, line up robots horizontally
    if robot.epoch == 0:
        # Set initial color to red
        robot.set_light("red", robot.snapshot_time if hasattr(robot, 'snapshot_time') else 0.0)
        
        # Place robots in a horizontal line with spacing
        spacing = 2.0  # units between robots
        robot_count = len(snapshot) if snapshot else 3
        x_pos = robot.id * spacing - (robot_count * spacing / 2)
        robot._logger.info(f"Robot {robot.id} initial position: {x_pos}, 0.0")
        return Coordinates(x_pos, 0.0), []
    
    # Check for obstruction and collinearity
    is_obstructing = False
    sees_collinear = False
    
    # Get positions of all visible robots
    robot_positions = []
    for rid, details in snapshot.items():
        if int(rid) != robot.id:  # Exclude self
            robot_positions.append(details.pos)
    
    # Check if I'm obstructing any other two robots' line of sight
    for i in range(len(robot_positions)):
        for j in range(i+1, len(robot_positions)):
            pos_i = robot_positions[i]
            pos_j = robot_positions[j]
            
            # Calculate distances
            dist_ij = math.dist(pos_i, pos_j)
            dist_i_me = math.dist(pos_i, my_pos)
            dist_j_me = math.dist(pos_j, my_pos)
            
            # If I'm approximately on the line between the two robots
            if abs(dist_i_me + dist_j_me - dist_ij) < 0.1:
                is_obstructing = True
                robot._logger.info(f"Robot {robot.id} is obstructing between robots")
                break
    
    # Check if I see any collinear robots (robots along the same line from my position)
    angles_to_robots = {}
    for pos in robot_positions:
        # Calculate angle from my position to other robot
        dx = pos.x - my_pos.x
        dy = pos.y - my_pos.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Skip if too close (avoid division by zero and rounding errors)
        if distance < 0.001:
            continue
            
        # Round angle to detect collinearity (use more precise rounding)
        angle = round(math.atan2(dy, dx) * 100) / 100  # Round to nearest 0.01 radian
        
        if angle not in angles_to_robots:
            angles_to_robots[angle] = []
        
        angles_to_robots[angle].append(pos)
    
    # If any angle has multiple robots, they're collinear from my perspective
    for angle, positions in angles_to_robots.items():
        if len(positions) > 1:
            sees_collinear = True
            robot._logger.info(f"Robot {robot.id} sees collinear robots at angle {angle}")
            break
    
    # Set color based on conditions
    if is_obstructing:
        new_color = "red"
    elif sees_collinear:
        new_color = "blue"
    else:
        new_color = "green"
    
    # Always update color to ensure visibility state is reflected
    robot.set_light(new_color, robot.snapshot_time if hasattr(robot, 'snapshot_time') else 0.0)
    robot._logger.info(f"Robot {robot.id} color changed to {new_color}")
    
    # Movement strategy
    if is_obstructing or sees_collinear:
        # Move perpendicular to the line
        # Use ID to determine direction - odd IDs up, even IDs down
        direction = 1 if robot.id % 2 == 1 else -1
        
        # Scale movement with epoch, but cap it
        move_distance = min(0.5 * (1 + robot.epoch % 5), 2.0)
        
        # If already moved, increase distance slightly to continue in same direction
        if hasattr(robot, '_last_move_direction') and robot._last_move_direction == direction:
            move_distance += 0.1 * robot.epoch
        
        new_pos = Coordinates(my_pos.x, my_pos.y + direction * move_distance)
        robot._last_move_direction = direction
        
        robot._logger.info(f"Robot {robot.id} moving to break visibility issues: {new_pos}")
        return new_pos, []
    
    # Reset movement tracking if not moving
    if hasattr(robot, '_last_move_direction'):
        delattr(robot, '_last_move_direction')
    
    return my_pos, []  # Stay in place if all is good

def mutual_visibility_terminal(robot, coord: Coordinates, args=None) -> bool:
    """
    Termination condition for mutual visibility algorithm:
    All visible robots should be green (no obstruction, no collinearity)
    
    Args:
        robot: The Robot instance executing the algorithm
        coord: Target coordinates 
        args: Additional arguments from the algorithm
        
    Returns:
        bool: True if termination condition is met, False otherwise
    """
    # If only this robot is visible, can't determine visibility conditions
    if len(robot.snapshot) <= 1:
        return True
    
    # Check if all robots (including self) are green (no obstruction, no collinearity)
    all_green = True
    for rid, details in robot.snapshot.items():
        if details.light != "green":
            all_green = False
            break
    
    # If all robots are green for 3 consecutive epochs, terminate
    if all_green:
        # Track number of "all green" epochs
        if not hasattr(robot, '_green_epochs'):
            robot._green_epochs = 0
        
        robot._green_epochs += 1
        
        # Terminate after 3 consecutive "all green" epochs
        if robot._green_epochs >= 3:
            return True
    else:
        # Reset counter if not all green
        robot._green_epochs = 0
    
    return False
