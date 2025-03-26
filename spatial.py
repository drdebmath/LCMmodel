from collections import defaultdict
from type_defs import Coordinates

class PositionGrid:
    """Optimized spatial partitioning for visibility checks"""
    def __init__(self, cell_size: float):
        self.cell_size = cell_size
        self.grid = defaultdict(set)  # Fixed: should be defaultdict(set)
        
    def update_position(self, robot_id: int, pos: Coordinates):  # Fixed typo in method name
        cell_x = int(pos.x // self.cell_size)
        cell_y = int(pos.y // self.cell_size)
        self.grid[(cell_x, cell_y)].add(robot_id)
        
    def get_nearby(self, pos: Coordinates, radius: float) -> set[int]:  # Fixed method name
        nearby = set()
        min_x = int((pos.x - radius) // self.cell_size)
        max_x = int((pos.x + radius) // self.cell_size)
        min_y = int((pos.y - radius) // self.cell_size)
        max_y = int((pos.y + radius) // self.cell_size)
        
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                nearby.update(self.grid.get((x, y), set()))
        return nearby

def round_coordinates(coord: Coordinates, precision: int):  # Fixed function name
    return Coordinates(round(coord.x, precision), round(coord.y, precision))
