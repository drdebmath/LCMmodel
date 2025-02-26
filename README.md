# Asynchronous Scheduler

A simulator for Look-Compute-Move robots in the Asynchronous System
This code is not intended to be run in a production environment because the custom algorithm feature allows for arbitrary code execution

## Installing required packages

`pip install -r requirements.txt`

## Running the Simulator

### Windows

`python run.py`

### MacOS / Linux

`python3 run.py`

The configuration takes the following variables.

- number of robots
  - Number of robots to generate random positions for
- initial positions (can be random) : list of points in 2D
  - Initial robot positions that are randomly generated unless specified by the user
- speed of robots : number
- type of scheduler (default: async)
- probability distribution (default: exponential)
- visibility radius : number
  - Visibility radius of the robots
- orientation of robots: same or random (input as a list of triple(translation, rotation, reflection)) (NOT IMPLEMENTED)
- multiplicity detection
  - Detects if multiple robots are in the same point
- colors (NOT IMPLEMENTED)
- obstructed visibility (NOT IMPLEMENTED)
- rigidity
  - Rigid movement allows the robot to reach it's destination when in the MOVE state.
  - Non-rigid movement means the robot will fall short of it's destination when in the MOVE state.

## Default Configuration

- Unlimited visibility
- No Multiplicity detection
- Ignore Collisions
- Transparent robots
- Rigid movement
