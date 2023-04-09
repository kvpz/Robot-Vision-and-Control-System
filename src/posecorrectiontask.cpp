#include "posecorrectiontask.hpp"

PoseCorrectionTask::PoseCorrectionTask()
   : Task(TaskType::POSECORRECTION, POSECORRECTIONTASK_PRIORITY)
{

    
}

void PoseCorrectionTask::notStarted(std::shared_ptr<Map> map, 
                                    std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    status = TaskStatus::INPROGRESS;
    // slow down robot motor PWM speed
    // check to see how far off the robot's orientation is to the endpoint required orientation

}

void PoseCorrectionTask::inProgress(std::shared_ptr<Map> map, 
                                    std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{        
    //navigator->isRobotOriented(map);
    // assign robot new state depending on its orientation relative to waypoint
    switch(navigator->getRobotOrientationAtEndpoint(map)) {
        case ORIENTED:
            status = TaskStatus::COMPLETE;
            nextRobotState = STOP;
            correcting_orientation = false;
            break;
        case OFF_TO_RIGHT:
            status = TaskStatus::INPROGRESS;
            nextRobotState = ROTATE_CCW;
            correcting_orientation = true;
            break;
        case OFF_TO_LEFT:
            status = TaskStatus::INPROGRESS;
            nextRobotState = ROTATE_CW;
            correcting_orientation = true;
            break;
    }
}
    
void PoseCorrectionTask::suspended(std::shared_ptr<Map> map, 
                                   std::shared_ptr<Navigator> navigator, 
                                   RobotState& nextRobotState, TaskType& nextTaskType) 
{
    // orient task cannot be suspended.

}

void PoseCorrectionTask::complete(std::shared_ptr<Map> map, 
                                  std::shared_ptr<Navigator> navigator, 
                                  RobotState& nextRobotState, TaskType& nextTaskType) 
{
    nextTaskType = NA;
    nextRobotState = STOP;
    //task_queue.pop();
    //task_queue.top().setStatus(INPROGRESS);
}