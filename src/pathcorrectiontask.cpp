#include "pathcorrectiontask.hpp"

PathCorrectionTask::PathCorrectionTask()
    : Task(PATHCORRECTION)
{


}

void PathCorrectionTask::notStarted(Map* map, Navigator* navigator, RobotState& nextRobotState)
{
    setStatus(INPROGRESS);
}

void PathCorrectionTask::inProgress(Map* map, Navigator* navigator, RobotState& nextRobotState)
{
    double destX2 = destination.getX();
    double destY2 = destination.getY();
    double robotX2 = map->getRobotCurrentXCoordinatePoint();
    double robotY2 = map->getRobotCurrentYCoordinatePoint();
    RobotPoseToWaypoint rposetoway = navigator->isRobotOnPath(map, robotX2, robotY2, destX2, destY2);

    // assign robot a task depending on orientation relative to waypoint
    switch(rposetoway) {
    case NEAR:
        setStatus(COMPLETE);
        nextRobotState = STOP;
        correcting_position = false;
        break;
    case ON_PATH:
        setStatus(COMPLETE);
        //robotState = MOVE_FORWARD;
        nextRobotState = STOP;
        correcting_position = false;
        break;
    case OFF_PATH:
        setStatus(INPROGRESS);
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

void PathCorrectionTask::suspended(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType)
{
    //correction task cannot be suspended.

}

void PathCorrectionTask::complete(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType)
{
    //nextRobotState = STOP;
    //task_queue.pop();
    //task_queue.top().setStatus(INPROGRESS);
}