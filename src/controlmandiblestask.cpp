#include "controlmandiblestask.hpp"

ControlMandiblesTask::ControlMandiblesTask()
    : Task(TaskType::CONTROLMANDIBLES, OBJECTSEARCHTASK_PRIORITY)
{
    
}

void ControlMandiblesTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState)
{
    // figure out what mandibles need to be moved
    // figure out current state of robot mandibles
}

void ControlMandiblesTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState)
{
    
}

void ControlMandiblesTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void ControlMandiblesTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    // mandibles should always be closed 
}