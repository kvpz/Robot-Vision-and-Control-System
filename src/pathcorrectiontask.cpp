#include "pathcorrectiontask.hpp"

PathCorrectionTask::PathCorrectionTask()
    : Task(PATHCORRECTION), correcting_position(false), angleToDestTolerance(ORIENTATION_RANGE_TOLERANCE)
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
    double destX2 = map->getNextDestinationXY().getX();
    double destY2 = map->getNextDestinationXY().getY();
    double robotX2 = map->RobotX();
    double robotY2 = map->RobotY();
    RobotPoseToWaypoint rposetoway = navigator->isRobotOnPath(map);

    // assign robot a task depending on orientation relative to waypoint
    switch(rposetoway) {
        case NEAR:
            status = TaskStatus::COMPLETE;
            nextRobotState = STOP;
            correcting_position = false;
            break;
        case ON_PATH:
            status = TaskStatus::COMPLETE;
            //robotState = MOVE_FORWARD;
            nextRobotState = STOP;
            correcting_position = false;
            break;
        case OFF_PATH:
            status = TaskStatus::INPROGRESS;
            if(correcting_position == false) {
                // decide whether to rotate CW or CCW
                if(navigator->getAngleToDestination() > angleToDestTolerance) 
                    nextRobotState = ROTATE_CCW;
                else
                    nextRobotState = ROTATE_CW;
        
                correcting_position = true;
            }
            else {
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