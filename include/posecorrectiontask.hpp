#ifndef POSECORRECTIONTASK_HPP
#define POSECORRECTIONTASK_HPP
#include "task.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "navigator.hpp"
#include "map.hpp"
#include "settings.hpp"

class PoseCorrectionTask : public Task
{
public:
    PoseCorrectionTask();

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

private:

    bool correcting_orientation = false;

};

#endif