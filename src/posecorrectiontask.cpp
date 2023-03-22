#include "posecorrectiontask.hpp"

PoseCorrectionTask::PoseCorrectionTask()
{

    
}

void PoseCorrectionTask::notStarted(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    setStatus(INPROGRESS);
}

void PoseCorrectionTask::inProgress(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = navigator->isRobotOriented(map, getEndpointDesiredOrientation());
        
    // assign robot new state depending on its orientation relative to waypoint
    switch(robotOrientationAtEndpoint) {
        case ORIENTED:
            setStatus(COMPLETE);
            nextRobotState = STOP;
            correcting_orientation = false;
            break;
        case OFF_TO_RIGHT:
            setStatus(INPROGRESS);
            nextRobotState = ROTATE_CCW;
            correcting_orientation = true;
            break;
        case OFF_TO_LEFT:
            setStatus(INPROGRESS);
            nextRobotState = ROTATE_CW;
            correcting_orientation = true;
            break;
    }
}
    
void PoseCorrectionTask::suspended(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) 
{
    // orient task cannot be suspended.

}

void PoseCorrectionTask::complete(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) 
{
    //nextRobotState = STOP;
    //task_queue.pop();
    //task_queue.top().setStatus(INPROGRESS);
}