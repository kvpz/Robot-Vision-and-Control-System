#include "posecorrectiontask.hpp"

PoseCorrectionTask::PoseCorrectionTask()
   : Task(POSECORRECTION)
{

    
}

void PoseCorrectionTask::notStarted(std::shared_ptr<Map> map, 
                                    std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    status = TaskStatus::INPROGRESS;
}

void PoseCorrectionTask::inProgress(std::shared_ptr<Map> map, 
                                    std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{        
    navigator->isRobotOriented(map);
    // assign robot new state depending on its orientation relative to waypoint
    switch(navigator->getRobotOrientationToEndpoint(map)) {
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