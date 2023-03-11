#ifndef ROBOT_HPP
#define ROBOT_HPP
#include "waypoints.hpp"
#include "task.hpp"
#include "comms.hpp"
using namespace std;
using namespace ROBOTASKS;

#define ROBOTDEBUG true

enum RobotPoseToWaypoint {
			  NEAR, ON_PATH, OFF_PATH, NONE
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
}

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

class Robot {
public:
  Robot();
  void run();
  double getRobotAngleToPoint(const Robot&, double x, double y) const;
  void move_forward();
  void move_backward();
  void move_left();
  void move_right();
  void rotate_CW(); 
  void rotate_CCW(); 
  void stop();
  RobotPoseToWaypoint isRobotOnPath(double robotX, double robotY, double destX, double destY);
  void printStatus();

  // getters (inlined)
  inline double getX() const { return currentLocation.getX(); }
  inline double getY() const { return currentLocation.getY(); }
  inline RobotState getState() const { return state; }
  inline double getOrientation() const { return currentOrientation; }
  inline double getAngleToDestination() const { return angleToDest; }
  inline bool isNearEndpoint() const { return nearEndpoint; }

  // setters (inlined)
  void setCurrentXY(double x, double y) {
    currentLocation.setX(x);
    currentLocation.setY(y);
  }
  
  void setState(RobotState newState) {
    state = newState;
  }

  void setOrientation(double o)
  {
    currentOrientation = o;
  }
  
  void setIsNearEndpoint(bool b) { nearEndpoint = b; }

private:
  RobotState state = STOP;
  Waypoint currentLocation;
  double currentOrientation; // (gyro) orientation
  double velocity;
  double currentAngle; // relative to starting position
  Comms* comport;
  RobotPoseToWaypoint robotPoseToWaypoint = NONE;
  double angleToDest;
  bool nearEndpoint = false;
};

#endif

/*
  Notes:

  Think of current location as the location on the xy-grid. 

  This of the current Position as a superset of the current location data
  that also provides data about the robot's position in 3D space. 

  A robot will have a task to complete.

  There can be a data structure that has timestamps as keys and values of Tasks. 

  
 */
