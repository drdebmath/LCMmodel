class Robot {
  static ROBOT_SIZE = 10;
  static ROBOT_X_POS_FACTOR = 1;
  static ROBOT_Y_POS_FACTOR = -1;

  static STATE_COLOR_MAP = {
    WAIT: "#E4A125", // yellow
    LOOK: "#1A8FE3", // blue
    MOVE: "#008148", // green
    INACTIVE: "#000000",
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
  }

  /**
   * Sets the robot's color (light).
   * @param {string} newColor
   */
  setColor(newColor) {
    this.color = newColor;
  }

  getColor() {
    // Return the current color if set; otherwise, fall back on the default mapping.
    return this.color || Robot.STATE_COLOR_MAP[this.state];
  }

  static setRobotSize(size) {
    Robot.ROBOT_SIZE = size;
  }
}

export default Robot;
