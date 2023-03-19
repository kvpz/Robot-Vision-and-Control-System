#ifndef TRAVELTASK_HPP
#define TRAVELTASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "utility.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"

#define DEBUG_TRAVELTASK false

class Robot;

class TravelTask : public Task
{
public:
    TravelTask();

    // Task subtasks

    virtual void notStarted(Robot* robot) override;

    virtual void inProgress(Robot* robot) override;

    virtual void suspended() override;

    /*
        When a robot has completed traveling to its endpoint, it then needs to 
        correct its orientation so that it matches the orientation required
        by the endpoint pose.
    */
    virtual void complete() override;

    //travelTaskSuspendedState(task);
    //travelTaskCompleteState(task);


private:
    RobotState robotState; // ex. MOVE_BACKWARD
};

#endif