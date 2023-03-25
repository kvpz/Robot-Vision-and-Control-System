#ifndef PATHCORRECTIONTASK_HPP
#define PATHCORRECTIONTASK_HPP
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "map.hpp"

#define ORIENTATION_RANGE_TOLERANCE 2.0

class PathCorrectionTask : public Task
{
public:
    PathCorrectionTask();

    virtual void notStarted(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void inProgress(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void suspended(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

private:
    double angleToDestTolerance;
    bool correcting_position;        

};

#endif