#ifndef PATHCORRECTIONTASK_HPP
#define PATHCORRECTIONTASK_HPP
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "map.hpp"
#include "enums/travelDirection.hpp"
#include "settings.hpp"

class PathCorrectionTask : public Task
{
public:
    PathCorrectionTask();

    virtual void notStarted(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            std::shared_ptr<VisionData> visionData,
                            RobotState& nextRobotState) override;

    virtual void inProgress(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            std::shared_ptr<VisionData> visionData,
                            RobotState& nextRobotState) override;

    virtual void suspended(std::shared_ptr<Map> map, 
                           std::shared_ptr<Navigator> navigator, 
                           std::shared_ptr<VisionData> visionData,
                           RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          std::shared_ptr<VisionData> visionData,
                          RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void printTaskInfo() override;

private:
    bool correcting_position;        
};

#endif