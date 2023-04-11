#include "controlmandiblestask.hpp"

ControlMandiblesTask::ControlMandiblesTask(MandibleState desiredLeftState, 
                                           MandibleState desiredRightState,
                                           MandibleState currentLeftMandibleState,
                                           MandibleState currentRightMandibleState,
                                           XYPoint xy, double endpointOrientation,
                                           double actionPointProximityTolerance)
    : Task(TaskType::CONTROLMANDIBLES, CONTROLMANDIBLESTASK_PRIORITY)
{
    desiredLeftMandibleState = desiredLeftState;
    desiredRightMandibleState = desiredRightState;
    actionPoint = xy;
    actionPointOrientation = endpointOrientation;
    inActionState = false;
    actionStateSteps = 0;
    this->actionPointProximityTolerance = actionPointProximityTolerance;
}

void ControlMandiblesTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState)
{
    printTaskInfo(); //"ControlMandiblesTask::InProgress");        
    status = TaskStatus::INPROGRESS;
}

void ControlMandiblesTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState)
{
    // if robot is at the location this task requires it to be
    // and it is in the correct orientation, open mandible(s)
    // Proximity tolerance should be statically defined in settings.hpp
    bool isRobotNearActionPoint = 
        approximately(map->RobotX(), actionPoint.getX(), actionPointProximityTolerance) &&
        approximately(map->RobotY(), actionPoint.getY(), actionPointProximityTolerance);

    if(isRobotNearActionPoint || inActionState)
    {
        std::cout << "robot is near action point" << std::endl;
        // start open/close of mandibles
        // set task state variable to indicate mandibles are being actuated
        inActionState = true;

        // open/close mandibles
        if(desiredLeftMandibleState == MandibleState::open &&
           desiredRightMandibleState == MandibleState::open) {
            std::cout << "opening both mandibles" << std::endl;
            switch(actionStateSteps++) {
                case 0:
                    // open right mandible on state machine tick
                    // if action task step 1 not complete:
                    nextRobotState = RobotState::OPENING_RIGHT_MANDIBLE;
                    break;
                case 1: 
                    // open left mandible on next state machine tick
                    // if action task step 2 not complete
                    nextRobotState = RobotState::OPENING_LEFT_MANDIBLE;
                    status = TaskStatus::COMPLETE;
                    break;
            }
        }
        else if(desiredLeftMandibleState == MandibleState::open &&
                desiredRightMandibleState == MandibleState::closed) {
            // open right mandible on state machine tick
            // open left mandible on state machine tick
            // close right mandible on state machine tick
        }
        else if(desiredLeftMandibleState == MandibleState::closed &&
                desiredRightMandibleState == MandibleState::open) {
            // open right mandible on state machine tick
        }
        else if(desiredLeftMandibleState == MandibleState::closed &&
                desiredRightMandibleState == MandibleState::closed) {
            // open right mandible on state machine tick
            // open left mandible on state machine tick
            // close right mandible on state machine tick
        }
    }

    printTaskInfo(); //"ControlMandiblesTask::InProgress");        
}

void ControlMandiblesTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void ControlMandiblesTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState, TaskType& nextTaskType)
{

}

void ControlMandiblesTask::printTaskInfo() //std::string taskStateName)
{
    if(DEBUG_NAVIGATETOTASK) {
        //Task::printTaskInfo();
        //std::cout << "\n====== " << taskStateName << " =======\n" << std::endl;
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "action point: " << actionPoint << "\n";
        std::cout << "action point desired orientation: " << actionPointOrientation << "\n";
        //std::cout << "is robot at action point: " << isRobotAtEndpoint << "\n";
        std::cout << "is robot performing action: " << inActionState << "\n";
        std::cout << "action point proximity tolerance: " << actionPointProximityTolerance << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}