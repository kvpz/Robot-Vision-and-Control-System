#include "posecorrectiontask.hpp"

PoseCorrectionTask::PoseCorrectionTask(){}

// Task subtasks

void PoseCorrectionTask::notStarted(Map* map, Navigator* navigator, RobotState& robotState) 
{
    setStatus(INPROGRESS);
}

void PoseCorrectionTask::inProgress(Map* map, Navigator* navigator, RobotState& robotState) 
{
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = navigator->isRobotOriented(map, getEndpointOrientation());
        
    // assign robot new state depending on its orientation relative to waypoint
    switch(robotOrientationAtEndpoint) {
        case ORIENTED:
            setStatus(COMPLETE);
            robotState = STOP;
            correcting_orientation = false;
            break;
        case OFF_TO_RIGHT:
            setStatus(INPROGRESS);
            robotState = ROTATE_CCW;
            correcting_orientation = true;
            break;
        case OFF_TO_LEFT:
            setStatus(INPROGRESS);
            robotState = ROTATE_CW;
            correcting_orientation = true;
            break;
    }
}
    
void PoseCorrectionTask::suspended() 
{
    // orient task cannot be suspended.

}

void PoseCorrectionTask::complete() 
{
    //nextRobotState = STOP;
    //task_queue.pop();
    //task_queue.top().setStatus(INPROGRESS);
}