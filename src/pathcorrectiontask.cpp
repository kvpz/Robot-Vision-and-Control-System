#include "pathcorrectiontask.hpp"

PathCorrectionTask::PathCorrectionTask()
    : Task(PATHCORRECTION)
{
    correcting_position = false;
}

void PathCorrectionTask::notStarted(std::shared_ptr<Map> map, 
                                    std::shared_ptr<Navigator> navigator, 
                                    RobotState& nextRobotState)
{
    status = TaskStatus::INPROGRESS;
}

void PathCorrectionTask::inProgress(std::shared_ptr<Map> map, 
                                    std::shared_ptr<Navigator> navigator, 
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
                if(navigator->getIsTravelDirectionForward() && 
                    navigator->robotAngularDistanceToEndpoint(map, false) < ORIENTATION_RANGE_TOLERANCE) {
                    nextRobotState = ROTATE_CW;
                }
                else if (navigator->getIsTravelDirectionForward() && 
                    navigator->robotAngularDistanceToEndpoint(map, false) > -1.0 * ORIENTATION_RANGE_TOLERANCE){
                    nextRobotState = ROTATE_CCW;
                }
                else if(!navigator->getIsTravelDirectionForward() && 
                    navigator->robotAngularDistanceToEndpoint(map, true) < ORIENTATION_RANGE_TOLERANCE) {
                    nextRobotState = ROTATE_CW;
                }
                else if (!navigator->getIsTravelDirectionForward() && 
                    navigator->robotAngularDistanceToEndpoint(map, true) > -1.0 * ORIENTATION_RANGE_TOLERANCE){
                    nextRobotState = ROTATE_CCW;
                }
                else {
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
                                   RobotState& nextRobotState, TaskType& nextTaskType)
{
    //correction task cannot be suspended.

}

void PathCorrectionTask::complete(std::shared_ptr<Map> map, 
                                  std::shared_ptr<Navigator> navigator, 
                                  RobotState& nextRobotState, TaskType& nextTaskType)
{
    nextTaskType = NAVIGATETO;
    nextRobotState = STOP;
}