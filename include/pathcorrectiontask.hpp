#ifndef PATHCORRECTIONTASK_HPP
#define PATHCORRECTIONTASK_HPP
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "map.hpp"

class PathCorrectionTask : public Task
{
public:
    PathCorrectionTask();

    virtual void notStarted(Map* map, Navigator* navigator, RobotState& nextRobotState) override;

    virtual void inProgress(Map* map, Navigator* navigator, RobotState& nextRobotState) override;

    virtual void suspended(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

private:
    RobotState robotState;
    double angleToDestTolerance = 10.0;
    bool correcting_position = false;        

};

#endif