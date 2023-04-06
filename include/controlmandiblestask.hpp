#ifndef CONTROLMANDIBLESTASK_HPP
#define CONTROLMANDIBLESTASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "navigator.hpp"
#include "map.hpp"
#include "settings.hpp"

class ControlMandiblesTask : public Task
{
public:
    ControlMandiblesTask();

    virtual void notStarted(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            RobotState& nextRobotState) override;

    virtual void inProgress(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            RobotState& nextRobotState) override;

    virtual void suspended(std::shared_ptr<Map> map, 
                           std::shared_ptr<Navigator> navigator, 
                           RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          RobotState& nextRobotState, TaskType& nextTaskType) override;

private:
    


};

#endif