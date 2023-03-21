#ifndef PATHCORRECTIONTASK_HPP
#define PATHCORRECTIONTASK_HPP
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "map.hpp"

class PathCorrectionTask : public Task
{
public:
    PathCorrectionTask();

    virtual void notStarted(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void inProgress(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void suspended(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

private:
    double angleToDestTolerance = 10.0;
    bool correcting_position = false;        

};

#endif