#include "orientTask.hpp"

OrientTask::OrientTask(){}

// Task subtasks

void OrientTask::notStarted(Robot* robot) 
{
    setStatus(INPROGRESS);
}

void OrientTask::inProgress(Map* map, Navigator* navigator, RobotState& robotState) 
{
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = navigator->isRobotOriented(map->getRobotOrientation(), getEndpointOrientation());
        
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
    
void OrientTask::suspended() 
{
    // orient task cannot be suspended.

}

void OrientTask::complete() 
{
    //nextRobotState = STOP;
    //task_queue.pop();
    //task_queue.top().setStatus(INPROGRESS);
}