#include "correctionTask.hpp"

CorrectionTask::CorrectionTask(){}

// Task subtasks

void CorrectionTask::notStarted(Map* map, Navigator* navigator, RobotState& robotState)
{
    setStatus(INPROGRESS);
}

void CorrectionTask::inProgress(Map* map, Navigator* navigator, RobotState& robotState)
{
    double destX2 = destination.getX();
    double destY2 = destination.getY();
    double robotX2 = map->getRobotCurrentXCoordinatePoint();
    double robotY2 = map->getRobotCurrentYCoordinatePoint();
    RobotPoseToWaypoint rposetoway = navigator->isRobotOnPath(robotX2, robotY2, destX2, destY2);

    // assign robot a task depending on orientation relative to waypoint
    switch(rposetoway) {
    case NEAR:
        setStatus(COMPLETE);
        robotState = STOP;
        correcting_position = false;
        break;
    case ON_PATH:
        setStatus(COMPLETE);
        //robotState = MOVE_FORWARD;
        robotState = STOP;
        correcting_position = false;
        break;
    case OFF_PATH:
        setStatus(INPROGRESS);
        if(correcting_position == false) {
            // decide whether to rotate CW or CCW
            if(navigator->getAngleToDestination() > angleToDestTolerance) 
                robotState = ROTATE_CCW;
            else
                robotState = ROTATE_CW;
    
            correcting_position = true;
        }
        else {
            correcting_position = true;
        }

        break;
    }
}

void CorrectionTask::suspended()
{
    //correction task cannot be suspended.

}

void CorrectionTask::complete()
{
    //nextRobotState = STOP;
    //task_queue.pop();
    //task_queue.top().setStatus(INPROGRESS);
}