#ifndef ROBOTPOSETOWAYPOINT_HPP
#define ROBOTPOSETOWAYPOINT_HPP
#include <string> 

enum RobotPoseToWaypoint
{
    NEAR, 
    NEARNOTDESIREDORIENTATION,
    ON_PATH, 
    OFF_PATH, 
    NONE
};

enum RobotPosition {
    ROBOT_NEAR_ENDPOINT,
    ROBOT_MISALIGNED_WITH_WAYPOINT,
    ROBOT_ON_WAYPOINT_PATH,
    ROBOT_OFF_WAYPOINT_PATH,
    ROBOT_POSITION_UNDEFINED
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