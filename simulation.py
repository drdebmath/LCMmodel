import sys
import os
# Add the Downloads folder to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Fix: Remove duplicate import
from scheduler import Scheduler
import logging
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

def main():
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("SwarmSim")
    
    # Reduced robot count for testing (1000 may be too heavy)
    num_robots = 100  # Reduced from 1000 for initial testing
    visibility_radius = 5.0
    area_size = 50  # Reduced area for testing
    
    # Generate random initial positions
    np.random.seed(42)
    initial_positions = np.random.uniform(-area_size/2, area_size/2, (num_robots, 2)).tolist()
    
    # Create scheduler
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
        logger.error(f"Failed to initialize scheduler: {str(e)}")
        return

    # Run simulation
    logger.info("Starting simulation...")
    while True:
        result = scheduler.handle_event()
        if result == -1:
            break
    
    # Visualize if we have snapshots
    if hasattr(scheduler, 'visualization_snapshots') and scheduler.visualization_snapshots:
        visualize(scheduler)
    else:
        logger.warning("No visualization data collected")

def visualize(scheduler):
    """Improved visualization with error handling"""
    try:
        print("Preparing visualization...")
        fig, ax = plt.subplots(figsize=(10, 10))  # Slightly smaller figure
        
        # Adjust visualization parameters based on robot count
        robot_count = len(scheduler.robots)
        marker_size = 6 if robot_count <= 100 else 3
        frame_skip = max(1, robot_count // 100)  # Auto-adjust frame skipping
        
        # Color mapping
        fault_colors = {
            None: 'blue', 'crash': 'red', 
            'byzantine': 'orange', 'delay': 'yellow'
        }
        
        def update(frame):
            try:
                ax.clear()
                time, snapshot = scheduler.visualization_snapshots[frame * frame_skip]
                
                # Draw robots
                for robot_id, details in snapshot.items():
                    robot = scheduler.robots[robot_id]
                    ax.plot(details.pos.x, details.pos.y, 'o',
                           color=fault_colors.get(robot.fault_type, 'purple'),
                           markersize=marker_size,
                           alpha=0.7)
                
                ax.set_title(f"Time: {time:.2f} | Robots: {len(snapshot)}")
                ax.grid(True)
                
            except Exception as e:
                print(f"Error rendering frame: {str(e)}")

        # Create animation with blit=True for better performance
        ani = FuncAnimation(fig, update, 
                          frames=len(scheduler.visualization_snapshots)//frame_skip,
                          interval=100,
                          blit=False)
        
        plt.show()
        
    except Exception as e:
        print(f"Visualization failed: {str(e)}")

if __name__ == "__main__":
    main()
