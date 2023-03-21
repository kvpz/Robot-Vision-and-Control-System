#ifndef POSECORRECTIONTASK_HPP
#define POSECORRECTIONTASK_HPP
#include "task.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "navigator.hpp"
#include "map.hpp"

class PoseCorrectionTask : public Task
{
public:
    PoseCorrectionTask();

    virtual void notStarted(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void inProgress(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState) override;
      
    virtual void suspended(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

private:

    bool correcting_orientation = false;

};

#endif