#ifndef ORIENTTASK_HPP
#define ORIENTTASK_HPP
//#include "robot.hpp"
#include "task.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"

//class Robot;

class OrientTask : public Task
{
public:
    OrientTask();

    // Task subtasks

    virtual void notStarted(Map* map, Navigator* navigator, RobotState& robotState) override;

    virtual void inProgress(Map* map, Navigator* navigator, RobotState& robotState) override;
      
    virtual void suspended() override;

    virtual void complete() override;

private:
    RobotState robotState; // ex. MOVE_BACKWARD
    bool correcting_orientation = false;

};

#endif