#ifndef TRAVELTASK_HPP
#define TRAVELTASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"

#define DEBUG_TRAVELTASK false

class Robot;

class TravelTask : public Task
{
public:
    TravelTask();

    // Task subtasks

    virtual void notStarted(Map* map, Navigator* navigator, RobotState& robotState) override;

    virtual void inProgress(Map* map, Navigator* navigator, RobotState& robotState) override;

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
    RobotPoseToWaypoint rPoseToWaypoint;

};

#endif