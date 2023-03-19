#ifndef ROBOT_HPP
#define ROBOT_HPP
#include "waypoint.hpp"
#include "taskmanager.hpp"
//#include "task.hpp"
#include "comms.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
//#include "utility.hpp"
#include "map.hpp"
#include "navigator.hpp"

using namespace std;

#define ROBOTDEBUG true

//class TaskManager;

class Robot {
public:
    Robot() 
    { 
        comport = new Comms("/dev/ttyACM0");
        robotPoseToWaypoint = NONE;
        state = STOP;
        taskManager = new TaskManager();
        navigator = new Navigator();
        map = new Map();
    }

    //void run();
    void run()
    {
        switch (state) {
          case MOVE_FORWARD:
            move_forward(); 
            break;
          case MOVE_LEFT:
            move_left();
            break;
          case MOVE_RIGHT:
              move_right();
            break;
          case ROTATE_CW:
            rotate_CW(); 
            break;
          case ROTATE_CCW:
            rotate_CCW(); 
            break;
          case MOVE_BACKWARD:
            move_backward();
            break;
          case STOP:
            stop();
            break;
        }
    }

    void move_forward() { comport->send_command("F"); }
    void move_backward() { comport->send_command("B"); }
    void move_left() { comport->send_command("L"); }
    void move_right() { comport->send_command("R"); }
    void rotate_CW() { comport->send_command("C"); }
    void rotate_CCW() { comport->send_command("Z"); }
    void stop() { comport->send_command("S"); }

    void printStatus() 
    {
        std::cout << "\n====== Robot Status ======\n";
        std::cout << "State: " << RobotStateToString(state) << "\n";
        std::cout << "current location: ("
                << map->getRobotCurrentXCoordinatePoint() << ", "
                << map->getRobotCurrentYCoordinatePoint() << ")\n";
        std::cout << "current orientation (yaw): " << map->getRobotOrientation() << "\n";
        std::cout << "robot pose relative to waypoint: " << printRobotPoseToWaypoint(robotPoseToWaypoint) << "\n";
        std::cout << "==========================\n";
        std::cout << std::endl;
    }

    // getters (inlined)
    inline double getX() const { return map->getRobotCurrentXCoordinatePoint(); }
    inline double getY() const { return map->getRobotCurrentXCoordinatePoint(); }
    inline RobotState getState() const { return state; }
    inline double getOrientation() const { return map->getRobotOrientation(); }
    inline double getAngleToDestination() const { return navigator->getAngleToDestination(); }
    inline TaskManager* getTaskManager() const { return taskManager; }
    inline bool hasTasks() const { return taskManager->hasTasks(); }
    // map related functions
    inline bool isNearEndpoint() const { return nearEndpoint; }
    inline RobotState getTravelDirection() { return state; } //travelDirection; }

    inline Map* getMap() { return map; }
    inline Navigator* getNavigator() { return navigator; }

    // setters (inlined)
    void setTravelDirection(RobotState travDir) { state = travDir; } //travelDirection = travDir; }

    void setState(RobotState newState) 
    {
      state = newState;
    }
    
    void updateRobotState(RobotState nextRobotState)
    {
        if(state != nextRobotState) {
            setState(nextRobotState);
            run();
        }
    }

    void setCurrentXY(double x, double y) 
    {
      map->setRobotCurrentCoordinate(x,y);
    }

    void setOrientation(double o)
    {
      map->setRobotOrientation(o);
    }
    
    void setIsNearEndpoint(bool b) { nearEndpoint = b; }

    void executeCurrentTask()
    {
      taskManager->executeCurrentTask(map, navigator);
      updateRobotState(taskManager->getNextRobotState());
    }

private:
    RobotState state = STOP;
    RobotState nextRobotState;

    Comms* comport;
    RobotPoseToWaypoint robotPoseToWaypoint = NONE;
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = NOTORIENTED;
    bool nearEndpoint = false;

    //Waypoint currentLocation;
    //double currentOrientation; // (gyro) orientation
    //RobotState travelDirection;

    TaskManager* taskManager;
    //TaskScheduler taskScheduler;

    Navigator* navigator;
    Map* map;


};


#endif
