#include "pathcorrectiontask.hpp"

PathCorrectionTask::PathCorrectionTask()
    : Task(PATHCORRECTION)
{
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

            if(approximately(map->getDestinationOrientation(), map->getRobotOrientation(), ORIENTATION_RANGE_TOLERANCE)) {
                    
            else if(navigator->getAngleToDestination(map) < 0.0) 
                nextRobotState = ROTATE_CCW;
            else
                nextRobotState = ROTATE_CW;
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