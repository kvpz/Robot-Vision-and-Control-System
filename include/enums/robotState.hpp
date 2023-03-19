#ifndef ROBOTSTATE_HPP
#define ROBOTSTATE_HPP

enum RobotState {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    ROTATE_CW,
    ROTATE_CCW,
    STOP
};

static std::string RobotStateToString(RobotState state) 
{
    switch(state) {
      case MOVE_FORWARD:
        return "MOVE_FORWARD";
      case MOVE_BACKWARD:
        return "MOVE_BACKWARD";
      case MOVE_LEFT:
        return "MOVE_LEFT";
      case MOVE_RIGHT:
        return "MOVE_RIGHT";
      case ROTATE_CW:
        return "ROTATE_CW";
      case ROTATE_CCW:
        return "ROTATE_CCW";
      case STOP:
        return "STOP";
      default:
        return "error";
    }
}

#endif