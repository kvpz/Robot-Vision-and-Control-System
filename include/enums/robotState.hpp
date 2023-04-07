#ifndef ROBOTSTATE_HPP
#define ROBOTSTATE_HPP

enum RobotState {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    ROTATE_CW,
    ROTATE_CCW,
    STOP,
    OPENING_RIGHT_RECEPTACLE,
    OPENING_LEFT_RECEPTACLE,
    CLOSING_RIGHT_RECEPTACLE,
    CLOSING_LEFT_RECEPTACLE,
    OPENING_LEFT_MANDIBLE,
    OPENING_RIGHT_MANDIBLE,
    CLOSING_LEFT_MANDIBLE,
    CLOSING_RIGHT_MANDIBLE
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
      case OPENING_RIGHT_RECEPTACLE:
        return "OPENING_RIGHT_RECEPTACLE";
      case OPENING_LEFT_RECEPTACLE:
        return "OPENING_LEFT_RECEPTACLE";
      case CLOSING_RIGHT_RECEPTACLE:
        return "CLOSING_RIGHT_RECEPTACLE";
      case CLOSING_LEFT_RECEPTACLE:
        return "CLOSING_LEFT_RECEPTACLE";
      case OPENING_LEFT_MANDIBLE:
        return "OPENING_LEFT_MANDIBLE";
      case OPENING_RIGHT_MANDIBLE:
        return "OPENING_RIGHT_MANDIBLE";
      case CLOSING_LEFT_MANDIBLE:
        return "CLOSING_LEFT_MANDIBLE";
      case CLOSING_RIGHT_MANDIBLE:
        return "CLOSING_RIGHT_MANDIBLE";
      case STOP:
        return "STOP";
      default:
        return "error";
    }
}

#endif