#ifndef ROBOT_HPP
#define ROBOT_HPP
#include "waypoints.hpp"
#include "task.hpp"
using namespace std;

#define DEBUG true

enum RobotState {
    IDLE,
    MOVE_FORWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    STOP
};

class Robot {
public:
  Robot(){}

  // getters
  inline double getX() { return currentPosition.getX(); }
  inline double getY() { return currentPosition.getY(); }
  inline RobotState getState() { return state; }
  
  // setters
  void setCurrentXY(double x, double y) {
    currentLocation.setX(x);
    currentLocation.setY(y);
  }
  
  void setState(RobotState newState) {
    state = newState;
  }

  void setWaypoint(Waypoint way){
    
  }

  void setTask(Task task)
  {
    
  }

  Task getTask()
  {
    return task;
  }
  
  void run() {
    switch (state) {
    case IDLE:
      
      break;
    case MOVE_FORWARD:
      move_forward(); 
      break;
    case MOVE_LEFT:


      break;
    case MOVE_RIGHT:


      break;
    case STOP:
      stop();

      break;
    }
  }

  void move_forward()
  {
    send_command("F");
  }
  
  void move_forward(double distance, double robot_speed_per_sec)
  {
    if(DEBUG) std::cout << "move forward" << std::endl;
    send_command("F");
    //std::cout << "Sleep duration: " << (long)(distance / robot_speed_per_sec * 1000.0) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)(distance / robot_speed_per_sec * 1000.0)));
    stop();
  }

  void move_backward(double distance, double robot_speed_per_sec)
  {
    if(DEBUG) std::cout << "move backward" << std::endl;
    send_command("B");
    //std::cout << "Sleep duration: " << (long)(distance / robot_speed_per_sec * 1000.0) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)(distance / robot_speed_per_sec * 1000.0)));
    stop();
  }

  void rotate_CW(double degrees, double robot_speed)
  {
    if(DEBUG) std::cout << "rotate_CW ( " << degrees << " )" << std::endl;
    send_command("C");
    //std::cout << "Sleep duration: " << (long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees)));
    stop();
  }

  void rotate_CCW(double degrees, double robot_speed)
  {
    if(DEBUG) std::cout << "rotate_CCW ( " << degrees << " )" << std::endl;
    send_command("Z");
    //std::cout << "Sleep duration: " << (long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees)));
    stop();
  }

  void stop()
  {
    send_command("S");
  }
    
private:
  RobotState state = IDLE;
  Task task; // current task robot must complete
  // start and end times would be good for detecting if a
  // path finished sooner or later than expected.
  //double start_time;
  //double end_time;
  Waypoint currentLocation;
  Waypoint currentPosition; // (gyro) orientation
  
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
