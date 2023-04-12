#include "followobjecttask.hpp"

FollowObjectTask::FollowObjectTask()
    : Task(TaskType::FOLLOWOBJECT, FOLLOWOBJECTTASK_PRIORITY)
{
    
}

void FollowObjectTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState)
{
    
}

void FollowObjectTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState)
{
    
}

void FollowObjectTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void FollowObjectTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void FollowObjectTask::printTaskInfo()
{
    if(DEBUG_FOLLOWOBJECTTASK) {
        Task::printTaskInfo(*this);
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}
