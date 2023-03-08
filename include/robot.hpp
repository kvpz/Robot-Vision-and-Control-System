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

/* 
   Check the current position of the robot relative to the destination.
   The robot's position relative to the waypoint:
   (1) before the waypoint
   (2) ahead of the waypoint
   (3) aligned with the waypoint
   (4) before the waypoint but off to the left
   (5) before the waypoint but off to the right
   (6) near the waypoint
*/

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
  Robot()
  { 
    comport = new Comms("/dev/ttyACM0");
    robotPoseToWaypoint = NONE;
  }

  // getters
  inline double getX() const { return currentLocation.getX(); }
  inline double getY() const { return currentLocation.getY(); }
  inline RobotState getState() const { return state; }
  inline double getOrientation() const { return currentOrientation; }

  // setters
  void setCurrentXY(double x, double y) {
    currentLocation.setX(x);
    currentLocation.setY(y);
  }
  
  void setState(RobotState newState) {
    state = newState;
  }

/*
  void setTask(Task& t, std::string name)
  {
    task = *(new Task(t.getTaskType(), name));
  }
*/
  void setOrientation(double o)
  {
    currentOrientation = o;
  }
  
  /*
  const Task getTask() 
  {
    return task;
  }
  */
  void run(Task& task) {
    switch (state) {
    case MOVE_FORWARD:
      move_forward(); 
      break;
    case MOVE_LEFT:
      //move_left();
      break;
    case MOVE_RIGHT:

      break;
    case ROTATE_CW:
      rotate_CW(task.getDesiredRobotYawPose()); //, ROBOT_SPEED_CM_PER_SEC);
      break;
    case ROTATE_CCW:
      rotate_CCW(task.getDesiredRobotYawPose()); //, ROBOT_SPEED_CM_PER_SEC);
      break;
    case MOVE_BACKWARD:

      break;
    case STOP:
      stop();
      break;
    }
  }

  double robotAngleToPoint(const Robot& robot, double x, double y) const
  {
    return angleToPoint(robot.getX(), robot.getY(), x, y, robot.getOrientation());
  }

  void move_forward()
  {
    comport->send_command("F");
  }
  
  void move_forward(double distance, double robot_speed_per_sec)
  {
    if(ROBOTDEBUG) std::cout << "move forward" << std::endl;
    comport->send_command("F");
    //std::cout << "Sleep duration: " << (long)(distance / robot_speed_per_sec * 1000.0) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)(distance / robot_speed_per_sec * 1000.0)));
    stop();
  }

  void move_backward(double distance, double robot_speed_per_sec)
  {
    if(ROBOTDEBUG) std::cout << "move backward" << std::endl;
    comport->send_command("B");
    //std::cout << "Sleep duration: " << (long)(distance / robot_speed_per_sec * 1000.0) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)(distance / robot_speed_per_sec * 1000.0)));
    stop();
  }

  void rotate_CW(double degrees) //, double robot_speed)
  {
    if(ROBOTDEBUG) std::cout << "rotate_CW ( " << degrees << " )" << std::endl;
    comport->send_command("C");
    //std::cout << "Sleep duration: " << (long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees)));
    stop();
  }

  void rotate_CCW(double degrees) //, double robot_speed)
  {
    if(ROBOTDEBUG) std::cout << "rotate_CCW ( " << degrees << " )" << std::endl;
    comport->send_command("Z");
    //std::cout << "Sleep duration: " << (long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees)));
    stop();
  }

  void stop()
  {
    comport->send_command("S");
  }
  
  void go_to_destination(double x_robot, double y_robot, double x_destination, double y_destination, double robot_current_angle)
  {
    double x_diff = x_destination - x_robot;
    double y_diff = y_destination - y_robot;
    double beta = 0.0;
    double theta = 0.0;
    double distance = 0.0;

    beta = atan(y_diff / x_diff) * 180.0 / (M_PI);
  
    if(x_diff > 0.0 && y_diff > 0.0){
      // first quadrant

    }
    else if(x_diff < 0.0 && y_diff > 0.0) {
      // second quadrant
      beta = beta + 180.0;
    }
    else if(x_diff < 0.0 && y_diff < 0.0) {
      // third quadrant
      beta = beta + 180.0;
    }
    else {
      // fourth quadrant
      beta = beta + 270.0;
    }

    theta = beta - robot_current_angle;
    distance = sqrt(x_diff * x_diff + y_diff * y_diff);
    std::cout << "robot current angle:  " << robot_current_angle << std::endl;
    std::cout << "distance: " << distance << std::endl;
    std::cout << "x2 - x1: " << x_diff << std::endl;
    std::cout << "y2 - y1: " << y_diff << std::endl;
    std::cout << "beta: " << beta << std::endl;
    std::cout << "theta: " << theta << std::endl;
  
    if(theta < 0.0) {
      // go clockwise
      rotate_CW(theta); //, ROBOT_360_ROTATE_TIME_MILLISEC);
    }
    else {
      // go counterclockwise
      rotate_CCW(theta); //, ROBOT_360_ROTATE_TIME_MILLISEC);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    move_forward(distance, ROBOT_SPEED_CM_PER_SEC);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  RobotPoseToWaypoint isRobotOnPath(double robotX, double robotY, double destX, double destY)
  {
    RobotPoseToWaypoint result = ON_PATH;
    // check if robot position (x,y) approximately near destination
    bool isYapproxnear = approximately(robotY, destY, 1.5, false);
    bool isXapproxnear = approximately(robotX, destX, 1.5, false);

    double angleToDest = robotAngleToPoint(*this, destX, destY);

    if(isYapproxnear && isXapproxnear) {
      // near the waypoint
      result = NEAR;
    }
    else if (angleToDest < 1.0) { // robotY > destY && robotX == destX
      // detect if drifting from path
      result = ON_PATH;
    }
    else {
      result = OFF_PATH;
    }    
    
    if(ROBOTDEBUG) {
      std::cout << "\n====== isRobotOnPath ======\n";
      std::cout << "result: " << printRobotPoseToWaypoint(result) << "\n";
      std::cout << "(isRobotOnPath) angle to dest: " << angleToDest << "\n";
      std::cout << "=============================\n" << std::endl;
    }

    robotPoseToWaypoint = result;
    return result;
  }

  RobotPoseToWaypoint robotPositionRelativeToWaypoint(double robotX, double robotY, double destX, double destY) const
  {
    RobotPoseToWaypoint result = ON_PATH;
    // check if robot position (x,y) approximately near destination
    bool isYapproxnear = approximately(robotY, destY, 1.5, false);
    bool isXapproxnear = approximately(robotX, destX, 1.5, false);

    double angleToDest = robotAngleToPoint(*this, destX, destY);

    if(angleToDest < (90.0 - 1.0)) {
      
    }
    else if (angleToDest > (90.0 + 1.0)) {
      
    }

    if(isYapproxnear && isXapproxnear) {
      // near the waypoint
      result = NEAR;
    }
    else if (angleToDest < 1.0) { // robotY > destY && robotX == destX
      // detect if drifting from path
      result = ON_PATH;
    }
    else if(robotY < destY && robotX < destX) {
      // before waypoint and off to the left
      result = BEFORE_LEFT;
    }
    else if(robotY < destY && robotX > destX) {
      // before waypoint and off to the right
      result = BEFORE_RIGHT;
    }
    else if(robotY > destY && robotX < destX) {
      // ahead of waypoint and to the left
      result = AFTER_LEFT;
    }
    else if(robotY > destY && robotX > destX) {
      // ahead of waypoint and to the right
      result = AFTER_RIGHT;
    }
    
    if(ROBOTDEBUG) {
      std::cout << "\n====== robotPostitionRelativeToWaypoint ======\n";
      std::cout << "result: " << printRobotPoseToWaypoint(result) << "\n";
      std::cout << "angle to dest: " << angleToDest << std::endl;
      std::cout << "==============================================\n" << std::endl;
    }

    return result;
  }

  void printStatus()
  {
    std::cout << "\n====== Robot Status ======\n";
    std::cout << "State: " << RobotStateToString(state) << "\n";
    std::cout << "current location: ("
	      << currentLocation.getX() << ", "
	      << currentLocation.getY() << ")\n";
    std::cout << "current orientation (yaw): " << currentOrientation << "\n";
    std::cout << "robot pose relative to waypoint: " << printRobotPoseToWaypoint(robotPoseToWaypoint) << "\n";
    std::cout << "==========================\n";
    std::cout << std::endl;
  }
  
private:
  RobotState state = STOP;
  //Task task; // current task robot must complete
  // start and end times would be good for detecting if a
  // path finished sooner or later than expected.
  Waypoint currentLocation;
  double currentOrientation; // (gyro) orientation
  double velocity;
  double currentAngle; // relative to starting position
  Comms* comport;
  RobotPoseToWaypoint robotPoseToWaypoint = NONE;
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
