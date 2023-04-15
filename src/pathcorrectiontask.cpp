#include "pathcorrectiontask.hpp"

PathCorrectionTask::PathCorrectionTask()
    : Task(TaskType::PATHCORRECTION, PATHCORRECTIONTASK_PRIORITY)
{
    correcting_position = false;
}

void PathCorrectionTask::notStarted(std::shared_ptr<Map> map, 
                                    std::shared_ptr<Navigator> navigator, 
                                    std::shared_ptr<VisionData> visionData,
                                    RobotState& nextRobotState)
{
    status = TaskStatus::INPROGRESS;
}

void PathCorrectionTask::inProgress(std::shared_ptr<Map> map, 
                                    std::shared_ptr<Navigator> navigator, 
                                    std::shared_ptr<VisionData> visionData,
                                    RobotState& nextRobotState)
{        
    // assign robot a task depending on orientation relative to waypoint
    switch(navigator->isRobotOnPath(map)) {
        case NEAR:
        case ON_PATH:
            status = TaskStatus::COMPLETE;
            nextRobotState = STOP;
            break;
        case OFF_PATH:
            status = TaskStatus::INPROGRESS;
            // decide whether to rotate CW or CCW
            if(correcting_position == false) {
                double angularDistanceToEndpoint = navigator->robotAngularDistanceToEndpoint(map, false);
                if(navigator->getTravelDirection() == TravelDirection::forward && 
                    angularDistanceToEndpoint < ORIENTATION_RANGE_TOLERANCE) {
                    std::cout << "(pathcorrection::inprogress) setting robot state to ROTATE_CW" << std::endl;
                    std::cout << "(pathcorrection::inprogress) angular distance to endpoint: " << angularDistanceToEndpoint << std::endl;
                    nextRobotState = ROTATE_CW;
                }
                else if (navigator->getTravelDirection() == TravelDirection::forward && 
                    angularDistanceToEndpoint > -1.0 * ORIENTATION_RANGE_TOLERANCE){
                    nextRobotState = ROTATE_CCW;
                }
                else if(navigator->getTravelDirection() == TravelDirection::backward && 
                    angularDistanceToEndpoint < ORIENTATION_RANGE_TOLERANCE) {
                    std::cout << "(pathcorrection::inprogress 2) setting robot state to ROTATE_CW" << std::endl;
                    std::cout << "(pathcorrection::inprogress 2) angular distance to endpoint: " << angularDistanceToEndpoint << std::endl;
                    nextRobotState = ROTATE_CW;
                }
                else if (navigator->getTravelDirection() == TravelDirection::backward && 
                    angularDistanceToEndpoint > -1.0 * ORIENTATION_RANGE_TOLERANCE){
                    nextRobotState = ROTATE_CCW;
                }
                else {
                    std::cout << "(pathcorrection::inprogress) status changed to COMPLETE" << std::endl;
                    status = TaskStatus::COMPLETE;
                    nextRobotState = STOP;
                }
                correcting_position = true;
            }

            break;
    }
    
}

void PathCorrectionTask::suspended(std::shared_ptr<Map> map, 
                                   std::shared_ptr<Navigator> navigator, 
                                   std::shared_ptr<VisionData> visionData,
                                   RobotState& nextRobotState, TaskType& nextTaskType)
{
    //correction task cannot be suspended.

}

void PathCorrectionTask::complete(std::shared_ptr<Map> map, 
                                  std::shared_ptr<Navigator> navigator, 
                                  std::shared_ptr<VisionData> visionData,
                                  RobotState& nextRobotState, TaskType& nextTaskType)
{
    nextTaskType = NAVIGATETO;
    nextRobotState = STOP;
}

void PathCorrectionTask::printTaskInfo()
{
    if(DEBUG_PATHCORRECTIONTASK) {
        Task::printTaskInfo(*this);
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}
