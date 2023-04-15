#include "controlwingstask.hpp"

ControlWingsTask::ControlWingsTask(WingState desiredLeftState, 
                                           WingState desiredRightState,
                                           XYPoint<double> xy,
                                           bool endpointOrientationRequired,
                                           double endpointOrientation,
                                           double actionPointProximityTolerance)
    : Task(TaskType::CONTROLWINGS, CONTROLWINGTASK_PRIORITY)
{
    desiredLeftWingState = desiredLeftState;
    desiredRightWingState = desiredRightState;
    actionPoint = xy;
    inActionState = false;
    actionStateSteps = 0;
    this->actionPointProximityTolerance = actionPointProximityTolerance;
    isEndpointOrientationRequired = endpointOrientationRequired;
    desiredEndpointOrientation = endpointOrientation;

    //std::cout << "desired left wing state: " << desiredLeftWingState << std::endl;
    //std::cout << "desired right wing state: " << desiredRightWingState << std::endl;

}

void ControlWingsTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState)
{
    //printTaskInfo();       
    status = TaskStatus::INPROGRESS;
}

void ControlWingsTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState)
{
    // if robot is at the location this task requires it to be
    // and it is in the correct orientation, open mandible(s)
    // Proximity tolerance should be statically defined in settings.hpp
    bool isRobotNearActionPoint = 
        approximately(map->RobotX(), actionPoint.getX(), actionPointProximityTolerance) &&
        approximately(map->RobotY(), actionPoint.getY(), actionPointProximityTolerance);
    bool isRobotNearActionPointOrientation = 
        approximately(map->getRobotOrientation(), desiredEndpointOrientation, 10.0);
    if((isRobotNearActionPoint && isRobotNearActionPointOrientation) || inActionState)
    {
        std::cout << "robot is near action point" << std::endl;
        // start open/close of mandibles
        // set task state variable to indicate mandibles are being actuated
        inActionState = true;

        // open/close mandibles
        if(desiredLeftWingState == WingState::open &&
           desiredRightWingState == WingState::open) {
            switch(actionStateSteps++) {
                case 0:
                    nextRobotState = RobotState::OPENING_RIGHT_WING;
                    break;
                case 1: 
                    nextRobotState = RobotState::OPENING_LEFT_WING;
                    status = TaskStatus::COMPLETE;
                    break;
            }
        }
        else if(desiredLeftWingState == WingState::open &&
                desiredRightWingState == WingState::closed) {

        }
        else if(desiredLeftWingState == WingState::closed &&
                desiredRightWingState == WingState::open) {
            // open right mandible on state machine tick
        }
        else if(desiredLeftWingState == WingState::closed &&
                desiredRightWingState == WingState::closed) {
            std::cout << "closing control wing state" << std::endl;
            switch(actionStateSteps++) {
                case 0:
                    // open right mandible on state machine tick
                    // if action task step 1 not complete:
                    nextRobotState = RobotState::CLOSING_LEFT_WING;
                    break;
                case 1: 
                    // open left mandible on next state machine tick
                    // if action task step 2 not complete
                    nextRobotState = RobotState::CLOSING_RIGHT_WING;
                    status = TaskStatus::COMPLETE;
                    break;
            }
        }
    }

    //printTaskInfo(); //"ControlMandiblesTask::InProgress");        
}

void ControlWingsTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void ControlWingsTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType)
{

}

void ControlWingsTask::printTaskInfo() //std::string taskStateName)
{
    if(DEBUG_CONTROLWINGSTASK) {
        Task::printTaskInfo(*this);
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "action point: " << actionPoint << "\n";
        std::cout << "is robot performing action: " << inActionState << "\n";
        std::cout << "action point proximity tolerance: " << actionPointProximityTolerance << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}