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
    Robot(double xpos, double ypos, double orientation);

    void run();

    void move_forward();
    void move_backward();
    void move_left();
    void move_right();
    void rotate_CW();
    void rotate_CCW();
    void stop();

    // getters (inlined)
    inline RobotState getState() const;

    inline double getX() const;
    inline double getY() const;
    inline double getOrientation() const;

    bool hasTasks();

    std::shared_ptr<TaskManager> getTaskManager();
    std::shared_ptr<Map> getMap();
    std::shared_ptr<Navigator> getNavigator();

    // setters (inlined)

    void setCurrentXY(double x, double y);

    void setOrientation(double o);

    void executeCurrentTask();

    void printStatus();

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
