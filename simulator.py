import sys
import os
import logging
import numpy as np
import asyncio
import websockets
import json
from typing import Dict, List
from dataclasses import dataclass

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from scheduler import Scheduler

# ==================
# WebSocket Server
# ==================
@dataclass
class RobotState:
    x: float
    y: float
    fault: str
    velocity: tuple = (0, 0)

class VisualizerServer:
    def __init__(self):
        self.connected = set()
        self.robot_states: Dict[int, RobotState] = {}

    async def broadcast(self):
        while True:
            if self.connected:
                message = json.dumps({
                    "robots": [
                        {"x": r.x, "y": r.y, "fault": r.fault}
                        for r in self.robot_states.values()
                    ]
                })
                for ws in self.connected:
                    await ws.send(message)
            await asyncio.sleep(0.05)  # 20 FPS

    async def handler(self, websocket):
        self.connected.add(websocket)
        try:
            async for _ in websocket:
                pass  # Keep connection open
        finally:
            self.connected.remove(websocket)

# ==================
# Main Simulation
# ==================
def main():
    # Setup
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("SwarmSim")
    
    # Configuration
    num_robots = 100
    visibility_radius = 5.0
    area_size = 50
    
    # WebSocket setup
    visualizer = VisualizerServer()
    loop = asyncio.get_event_loop()
    ws_server = websockets.serve(visualizer.handler, "localhost", 8001)
    loop.run_until_complete(ws_server)
    loop.create_task(visualizer.broadcast())

    # Initialize
    np.random.seed(42)
    initial_positions = np.random.uniform(-area_size/2, area_size/2, (num_robots, 2)).tolist()

    try:
        scheduler = Scheduler(
            logger=logger,
            seed=42,
            num_of_robots=num_robots,
            initial_positions=initial_positions,
            robot_speeds=1.0,
            algorithm="Gathering",
            visibility_radius=visibility_radius,
            obstructed_visibility=True,
            fault_prob=0.3,
            scheduler_type="ASYNC",
            sampling_rate=0.1
        )
    except Exception as e:
        logger.error(f"Initialization failed: {e}")
        return

    # Simulation loop
    logger.info("Starting simulation...")
    try:
        while True:
            result = scheduler.handle_event()
            
            # Update WebSocket data
            visualizer.robot_states = {
                id: RobotState(
                    x=robot.pos.x,
                    y=robot.pos.y,
                    fault=robot.fault_type or "NORMAL"
                )
                for id, robot in scheduler.robots.items()
            }

            if result == -1:
                break

    except KeyboardInterrupt:
        logger.info("Simulation stopped by user")

if __name__ == "__main__":
    try:
        main()
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
