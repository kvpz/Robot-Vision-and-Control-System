#ifndef ROBOTPOSETOWAYPOINT_HPP
#define ROBOTPOSETOWAYPOINT_HPP

enum RobotPoseToWaypoint
{
    NEAR, 
    NEARNOTDESIREDORIENTATION,
    ON_PATH, 
    OFF_PATH, 
    NONE
};

static std::string printRobotPoseToWaypoint(RobotPoseToWaypoint r) {
    switch(r) {
      case NEAR:
        return "NEAR";
      case ON_PATH:
        return "ON_PATH";
      case OFF_PATH:
        return "OFF_PATH";
      case NONE:
        return "NONE";
    }

    return "NA";
}

#endif