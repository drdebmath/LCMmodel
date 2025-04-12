class Robot {
  static ROBOT_SIZE = 10;
  static ROBOT_X_POS_FACTOR = 1;
  static ROBOT_Y_POS_FACTOR = -1;

  static STATE_COLOR_MAP = {
    WAIT: "#E4A125", // yellow
    LOOK: "#1A8FE3", // blue
    MOVE: "#008148", // green
    CRASH: "#FF0000", // red
    INACTIVE: "#000000",
  };

  /**
   * Represents a robot
   * @constructor
   * @param {number} x - X position
   * @param {number} y - Y position
   * @param {string} id - Robot's id
   * @param {string} color - Robot's color
   * @param {number} speed - Robot's speed
   * @param {number} multiplicity - # of Robots at position
   * @param {boolean} isCanvasCoordinates - flag to decide if coordinates are HTMLCanvas coordinates
   */
  constructor(x, y, id, color, speed, multiplicity = 1, isCanvasCoordinates = false) {
    /** @type {number} */ this.x = x;
    /** @type {number} */ this.y = y;
    /** @type {string} */ this.id = id;
    /** @type {string} */ this.color = color;
    /** @type {number} */ this.speed = speed;
    /** @type {number} */ this.multiplicity = multiplicity;

    /** @type {boolean} */ this.isCanvasCoordinates = isCanvasCoordinates;

    /** @type {string} */ this.state = "INACTIVE";
  }

  /**
   * @param {number} x
   * @param {number} y
   */
  setPosition(x, y) {
    this.x = x;
    this.y = y;
  }

  getCanvasPosition() {
    if (this.isCanvasCoordinates == true) {
      return [this.x, this.y];
    }

    return [this.x * Robot.ROBOT_X_POS_FACTOR, this.y * Robot.ROBOT_Y_POS_FACTOR];
  }

  getPosition() {
    if (this.isCanvasCoordinates == true) {
      return [this.x / Robot.ROBOT_X_POS_FACTOR, this.y / Robot.ROBOT_Y_POS_FACTOR];
    }

    return [this.x, this.y];
  }

  setState(state) {
    this.state = state;
  }

  getColor() {
    return Robot.STATE_COLOR_MAP[this.state];
  }

  static setRobotSize(size) {
    Robot.ROBOT_SIZE = size;
  }
}

export default Robot;
