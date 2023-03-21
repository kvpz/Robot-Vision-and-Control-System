#ifndef ROBOT_HPP
#define ROBOT_HPP
#include <memory>
#include "waypoint.hpp"
#include "taskmanager.hpp"
#include "comms.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "map.hpp"
#include "navigator.hpp"

using namespace std;

#define ROBOTDEBUG true

class Robot {
public:
    Robot(double xpos, double ypos) 
        : state(STOP)
      //: map(std::make_unique<Map>()), navigator(std::make_unique<Navigator>()), taskManager(std::make_unique<TaskManager>())
    { 
        comport = new Comms("/dev/ttyACM0");
        robotPoseToWaypoint = NONE;
        state = STOP;
        taskManager = std::make_unique<TaskManager>();
        navigator = std::make_unique<Navigator>();
        map = std::make_unique<Map>();
        map->setRobotCurrentCoordinate(xpos, ypos);
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
                << map->RobotY() << ")\n";
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
    inline std::unique_ptr<TaskManager> getTaskManager() { return std::move(taskManager); }
    inline bool hasTasks() { return taskManager->hasTasks(); }
    // map related functions
    inline bool isNearEndpoint() const { return nearEndpoint; }

    inline std::unique_ptr<Map> getMap() { return std::move(map); }
    inline std::unique_ptr<Navigator> getNavigator() { return std::move(navigator); }

    // setters (inlined)
    void setTravelDirection(RobotState travDir) { state = travDir; } //travelDirection = travDir; }

    void updateRobotState(RobotState nextRobotState)
    {
        if(state != nextRobotState) {
            state = nextRobotState;
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

    /*
      Robot sends data about itself to the task manager. 
      The task manager then executes the current task. 
      A task will update the state of the robot.
    */
    void executeCurrentTask()
    {
      //state = taskManager->executeCurrentTask(std::move(map), std::move(navigator));
      taskManager->executeCurrentTask(std::move(map), std::move(navigator), nextRobotState);
      updateRobotState(nextRobotState);
    }

private:
    RobotState state;
    RobotState nextRobotState; // better if TaskManager has this state and returns it to robot

    Comms* comport;
    RobotPoseToWaypoint robotPoseToWaypoint = NONE;
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = NOTORIENTED;
    bool nearEndpoint = false;

    //Waypoint currentLocation;
    //double currentOrientation; // (gyro) orientation
    //RobotState travelDirection;

    std::unique_ptr<TaskManager> taskManager;
    std::unique_ptr<Navigator> navigator;
    std::unique_ptr<Map> map;


};


#endif
