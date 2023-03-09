#ifndef ROBOT_HPP
#define ROBOT_HPP
#include "waypoints.hpp"
#include "task.hpp"
#include "comms.hpp"
using namespace std;
using namespace ROBOTASKS;

#define ROBOTDEBUG true

enum RobotPoseToWaypoint {
			  NEAR, BEFORE_LEFT, BEFORE_RIGHT,
			  AFTER_LEFT, AFTER_RIGHT, ON_PATH, OFF_PATH, NONE
};

static std::string printRobotPoseToWaypoint(RobotPoseToWaypoint r) {
  switch(r) {
    case NEAR:
      return "NEAR";
    case BEFORE_LEFT:
      return "BEFORE_LEFT";
    case BEFORE_RIGHT:
      return "BEFORE_RIGHT";
    case AFTER_LEFT:
      return "AFTER_LEFT";
    case AFTER_RIGHT:
      return "AFTER_RIGHT";
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
  }
}

class Robot {
public:
  Robot();
  void run(Task&);
  double robotAngleToPoint(const Robot&, double x, double y) const;
  void move_forward();
  void move_forward(double distance, double robot_speed_per_sec);
  void move_backward(double distance, double robot_speed_per_sec);
  void rotate_CW(double degrees); //, double robot_speed)
  void rotate_CCW(double degrees); //, double robot_speed)
  void stop();
  void go_to_destination(double x_robot, double y_robot, double x_destination, double y_destination, double robot_current_angle);
  RobotPoseToWaypoint isRobotOnPath(double robotX, double robotY, double destX, double destY);
  RobotPoseToWaypoint robotPositionRelativeToWaypoint(double robotX, double robotY, double destX, double destY);
  void printStatus();

  // getters (inlined)
  inline double getX() const { return currentLocation.getX(); }
  inline double getY() const { return currentLocation.getY(); }
  inline RobotState getState() const { return state; }
  inline double getOrientation() const { return currentOrientation; }
  inline double angleToDestination() const { return angleToDest; }

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
  
private:
  RobotState state = STOP;
  Waypoint currentLocation;
  double currentOrientation; // (gyro) orientation
  double velocity;
  double currentAngle; // relative to starting position
  Comms* comport;
  RobotPoseToWaypoint robotPoseToWaypoint = NONE;
  double angleToDest;
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
