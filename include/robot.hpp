#ifndef ROBOT_HPP
#define ROBOT_HPP
#include <memory>
#include "xypoint.hpp"
#include "taskmanager.hpp"
#include "comms.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "enums/attractionColors.hpp"
#include "map.hpp"
#include "navigator.hpp"
#include "settings.hpp"
#include "enums/mandibles.hpp"

enum class Speed {
    a='a', b,c,d,e,f
};

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
    void open_left_receptacle();
    void close_left_receptacle();
    void open_right_receptacle();
    void close_right_receptacle();
    void stop();

    // getters (inlined)
    inline RobotState getState() const;

    inline double getX() const;
    inline double getY() const;
    inline double getOrientation() const;

    bool hasTasks();

    std::shared_ptr<TaskManager> getTaskManager();
    std::shared_ptr<Map>         getMap();
    std::shared_ptr<Navigator>   getNavigator();

    MandibleState getLeftMandibleState() { return leftMandibleState; }
    MandibleState getRightMandibleState() { return rightMandibleState; }

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
    Speed speed;
    //RobotPoseToWaypoint robotPoseToWaypoint;
    //RobotOrientationAtEndpoint robotOrientationAtEndpoint;

    // 
    std::shared_ptr<TaskManager> taskManager;
    std::shared_ptr<Navigator> navigator;
    std::shared_ptr<Map> map;

    MandibleState leftMandibleState;
    MandibleState rightMandibleState;
};


#endif
