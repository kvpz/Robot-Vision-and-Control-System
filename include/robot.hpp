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

#define ROBOTDEBUG true

class Robot {
public:
    Robot(double xpos, double ypos, double orientation) 
        : state(STOP), robotPoseToWaypoint(NONE)
      //: map(std::make_shared<Map>()), navigator(std::make_unique<Navigator>()), taskManager(std::make_unique<TaskManager>())
    { 
        comport = std::make_unique<Comms>("/dev/ttyACM0");
        taskManager = std::make_shared<TaskManager>();
        navigator = std::make_unique<Navigator>();
        map = std::make_unique<Map>();
        map->setRobotCurrentCoordinate(xpos, ypos);
        map->setRobotOrientation(orientation);

        robotPoseToWaypoint = NONE;
        robotOrientationAtEndpoint = NOTORIENTED;

        std::cout << "======== Robot::Robot ========\n";
        std::cout << "taskManager address: " << &(*taskManager) << "\n";
        std::cout << "==============================\n" << std::endl;
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

    // getters (inlined)
    inline double getX() const { return map->getNextDestinationXY().getX(); }
    inline double getY() const { return map->getNextDestinationXY().getY(); }
    inline RobotState getState() const { return state; }
    inline double getOrientation() const { return map->getRobotOrientation(); }
    inline double getAngleToDestination() const { return navigator->getAngleToDestination(); }
    inline std::shared_ptr<TaskManager> getTaskManager() { return taskManager; }
    bool hasTasks() { return taskManager->hasTasks(); }

    //inline std::unique_ptr<Map> getMap() { return map; }
    inline std::shared_ptr<Navigator> getNavigator() { return navigator; }

    // setters (inlined)
    void setTravelDirection(RobotState travDir) { state = travDir; } //travelDirection = travDir; }

    void setCurrentXY(double x, double y) 
    {
      std::cout << "robot current location: " << x << " , " << y << "\n" << std::endl;
      map->setRobotCurrentCoordinate(x,y);
    }

    void setOrientation(double o)
    {
      map->setRobotOrientation(o);
    }
    
    /*
      Robot sends data about itself to the task manager. 
      The task manager then executes the current task. 
      A task will update the state of the robot.
    */
    void executeCurrentTask()
    {
      if(ROBOTDEBUG) {
        std::cout << "====== Robot::executeCurrentTask =======" << std::endl;
      }

      RobotState nextRobotState;
      //TODO: nextRobotState = taskManager->executeCurrentTask(map, navigator);
      taskManager->executeCurrentTask(map, navigator, nextRobotState);

      // change robot state if it is different from current state
      if(state != nextRobotState) {
          state = nextRobotState;
          run(); // alter robot state if it needs to be in a different
      }
    }

    void printStatus() 
    {
        std::cout << "\n====== Robot Status ======\n";
        std::cout << "State: " << RobotStateToString(state) << "\n";
        std::cout << "current location: ("
                  << map->RobotX() << ", "
                  << map->RobotY() << ")\n";
        std::cout << "current orientation (yaw): " << map->getRobotOrientation() << "\n";
        std::cout << "==========================\n";
        std::cout << std::endl;
    }

private:
    // communications
    std::unique_ptr<Comms> comport;

    // robot status indicators
    RobotState state;
    RobotPoseToWaypoint robotPoseToWaypoint;
    RobotOrientationAtEndpoint robotOrientationAtEndpoint;

    // 
    std::shared_ptr<TaskManager> taskManager;
    std::shared_ptr<Navigator> navigator;
    std::shared_ptr<Map> map;

};


#endif
