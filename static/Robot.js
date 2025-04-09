// Update the Robot.js file with these changes to improve white robot visibility

class Robot {
  static ROBOT_SIZE = 10;
  static ROBOT_X_POS_FACTOR = 1;
  static ROBOT_Y_POS_FACTOR = 1; // Changed from -1 to 1 to fix upside-down issue

  static STATE_COLOR_MAP = {
    WAIT: "#E4A125", // yellow
    LOOK: "#1A8FE3", // blue
    MOVE: "#008148", // green
    TERMINATED: "#888888", // gray for terminated robots
    INACTIVE: "#000000",
  };

  // Enhanced color map for named colors
  static NAMED_COLOR_MAP = {
    "red": "#FF0000",
    "blue": "#0000FF",
    "green": "#00FF00",
    "gray": "#888888",
    "white": "#FFFFFF",
    "black": "#000000",
    "yellow": "#FFFF00",
    "purple": "#800080",
    "orange": "#FFA500",
    "cyan": "#00FFFF",
    "magenta": "#FF00FF",
    "brown": "#A52A2A",
    "pink": "#FFC0CB"
  };

  // Add color change history for MutualVisibility algorithm
  static MUTUALVIS_COLORS = {
    RED: "#FF0000",
    GREEN: "#00FF00"
  };

  /**
   * Represents a robot.
   * @constructor
   * @param {number} x - X position.
   * @param {number} y - Y position.
   * @param {string} id - Robot's id.
   * @param {string} color - Robot's color (light).
   * @param {number} speed - Robot's speed.
   * @param {number} multiplicity - # of Robots at position.
   * @param {boolean} isCanvasCoordinates - flag to decide if coordinates are HTMLCanvas coordinates.
   */
  constructor(x, y, id, color, speed, multiplicity = 1, isCanvasCoordinates = false) {
    this.x = x;
    this.y = y;
    this.id = id;
    this.color = color; // this holds the current light color
    this.speed = speed;
    this.multiplicity = multiplicity;
    this.isCanvasCoordinates = isCanvasCoordinates;
    this.state = "INACTIVE";
    this.lastUpdateTime = Date.now();
    this.renderColor = this._getRenderColor(color);
    this.calculated_position = null; // Will hold the target position for movement
    
    // Add color change tracking for MutualVisibility
    this.colorHistory = [];
    this.lastColorChange = Date.now();
    this.consecutiveGreenTime = 0;
    
    // Debug log
    console.log(`Robot created: ID=${id}, Color=${color}, Position=[${x}, ${y}]`);
  }

  setPosition(x, y) {
    this.x = x;
    this.y = y;
  }

  getCanvasPosition() {
    if (this.isCanvasCoordinates === true) {
      return [this.x, this.y];
    }
    return [this.x * Robot.ROBOT_X_POS_FACTOR, this.y * Robot.ROBOT_Y_POS_FACTOR];
  }

  getPosition() {
    if (this.isCanvasCoordinates === true) {
      return [this.x / Robot.ROBOT_X_POS_FACTOR, this.y / Robot.ROBOT_Y_POS_FACTOR];
    }
    return [this.x, this.y];
  }

  setState(state) {
    this.state = state;
    // If a robot is terminated, it should show the terminated color
    if (state === "TERMINATED") {
      this.renderColor = Robot.STATE_COLOR_MAP.TERMINATED;
    }
  }

  /**
   * Sets the robot's color (light) and updates the render color.
   * Also tracks color changes for MutualVisibility algorithm.
   * @param {string} newColor
   */
  setColor(newColor) {
    const now = Date.now();
    const oldColor = this.color;
    
    // Track color change
    this.colorHistory.push({
      from: oldColor,
      to: newColor,
      timestamp: now
    });
    
    // Keep history manageable
    if (this.colorHistory.length > 100) {
      this.colorHistory.shift();
    }
    
    // Special handling for MutualVisibility algorithm
    if (newColor === Robot.MUTUALVIS_COLORS.GREEN) {
      if (oldColor === Robot.MUTUALVIS_COLORS.GREEN) {
        // Accumulate time spent continuously green
        this.consecutiveGreenTime += (now - this.lastColorChange);
      } else {
        // Reset consecutive green time when changing to green
        this.consecutiveGreenTime = 0;
      }
    } else {
      // Reset consecutive green time when changing to non-green
      this.consecutiveGreenTime = 0;
    }
    
    console.log(`Robot ${this.id} color changing from ${oldColor} to ${newColor} (consecutive green time: ${this.consecutiveGreenTime}ms)`);
    
    this.color = newColor;
    this.renderColor = this._getRenderColor(newColor);
    this.lastColorChange = now;
  }

  /**
   * Get the color to use for rendering this robot.
   * Converts named colors and handles special cases.
   * @private
   */
  _getRenderColor(colorName) {
    // If it's a hex color, return as is
    if (colorName && colorName.startsWith('#')) {
      return colorName.toUpperCase();
    }
    
    // Check for named colors - enhanced for MutualVisibility algorithm
    if (colorName && Robot.NAMED_COLOR_MAP[colorName.toLowerCase()]) {
      const hexColor = Robot.NAMED_COLOR_MAP[colorName.toLowerCase()];
      return hexColor.toUpperCase();
    }
    
    // Otherwise, return state-based color or default
    return Robot.STATE_COLOR_MAP[this.state] || "#000000";
  }

  getColor() {
    return this.renderColor;
  }

  /**
   * Check if this robot is currently moving
   * @returns {boolean}
   */
  isMoving() {
    return this.state === "MOVE";
  }

  /**
   * Check if this robot is terminated
   * @returns {boolean}
   */
  isTerminated() {
    return this.state === "TERMINATED";
  }

  /**
   * Get the time this robot has been continuously green
   * @returns {number} Time in milliseconds
   */
  getConsecutiveGreenTime() {
    if (this.color === Robot.MUTUALVIS_COLORS.GREEN) {
      return this.consecutiveGreenTime + (Date.now() - this.lastColorChange);
    }
    return 0;
  }

  /**
   * Check if this robot has been green for the specified duration
   * @param {number} duration - Duration in milliseconds
   * @returns {boolean}
   */
  hasBeenGreenFor(duration) {
    return this.getConsecutiveGreenTime() >= duration;
  }

  static setRobotSize(size) {
    Robot.ROBOT_SIZE = size;
  }
}

export default Robot;