import random
from dataclasses import dataclass
from typing import Optional, Literal

@dataclass
class Coordinates:
    x: float  # Fixed from 'xs float'
    y: float  # Fixed from 'ys float'

class Robot:
    def __init__(self, id: int, pos: Coordinates, fault: Optional[Literal["crash", "byzantine", "delay"]] = None):
        self.id = id
        self.pos = pos
        self.fault = fault
        self.frozen = False
        print(f"🤖 Robot {id} at ({pos.x:.1f}, {pos.y:.1f}) | Fault: {fault or 'None'}")  # Fixed formatting

    def move(self, target: Coordinates):
        if self.fault == "crash":
            print(f"💥 Robot {self.id} CRASHED and can't move")  # Fixed typo
            return
        if self.frozen or (self.fault == "delay" and random.random() < 0.3):
            print(f"⏸️ Robot {self.id} DELAYED (frozen)")
            self.frozen = True
            return
        print(f"🚀 Robot {self.id} moved from ({self.pos.x:.1f}, {self.pos.y:.1f}) → ({target.x:.1f}, {target.y:.1f})")
        self.pos = target

def main():
    print("=== ROBOT SIMULATION ===")
    robots = [
        Robot(0, Coordinates(0, 0)),
        Robot(1, Coordinates(1, 1), "crash"),
        Robot(2, Coordinates(2, 2), "byzantine"),
        Robot(3, Coordinates(3, 3), "delay")
    ]

    for step in range(1, 6):
        print(f"\n🔄 **STEP {step}**")
        for robot in robots:
            target = Coordinates(
                robot.pos.x + random.uniform(-1, 1),
                robot.pos.y + random.uniform(-1, 1)
            )
            robot.move(target)

if __name__ == "__main__":
    main()
