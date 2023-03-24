#include "navigatetotask.hpp"

NavigateToTask::NavigateToTask(double endpointOrientation, 
                               bool endpointOrientationRequirement) 
    : isRobotAtEndpoint(false), Task(NAVIGATETO)
{
    isEndpointOrientationRequired = endpointOrientationRequirement;
    endpointDesiredOrientation = endpointOrientation;

    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "\n======= NavigateToTask::NavigateToTask =======\n";
        std::cout << "isEndpointOrientationRequired: " << endpointOrientationRequirement << "\n";
        std::cout << "endpointOrientation: " << endpointOrientation << "\n";
        std::cout << "==============================================\n" << std::endl;
    }
}

NavigateToTask::NavigateToTask(XYPoint xy, double endpointOrientation, bool endpointOrientationRequired)
    : isRobotAtEndpoint(false), Task(NAVIGATETO)
{    
    endpoint.setX(xy.getX());
    endpoint.setY(xy.getY());
    endpointDesiredOrientation = endpointOrientation;
    isEndpointOrientationRequired = endpointOrientationRequired;
}


/*
    This function should initialize the map because the task 
    contains information about the 
*/
void NavigateToTask::notStarted(std::shared_ptr<Map> map, 
                                std::shared_ptr<Navigator> navigator, RobotState& nextRobotState)
{
    // At this point the robot should be told whether it should travel 
    // forward, backward, left, or right depending on its distance to the endpoint and current location
    double robotDistanceToEndpoint = distance(map->getRobotCurrentLocation(), map->getNextDestinationXY());
    double robot_orientation_minus_destination = navigator->getAngleToDestination();

    // move robot backward to a point if travel distance is short
    if(robotDistanceToEndpoint < 70.0 && robot_orientation_minus_destination > 130.0)
        nextRobotState = MOVE_BACKWARD;
    else
        nextRobotState = MOVE_FORWARD;

    // set next endpoint to navigate to in map
    map->setDestinationXY(endpoint.getX(), endpoint.getY());
    if(this->getIsEndpointOrientationRequired())
        map->setDestinationDesiredOrientation(endpointDesiredOrientation);

    // update task status
    setStatus(INPROGRESS);

    /*
        TODO:
        (1) implement optimal travel direction
        (2)  
    */
    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "\n====== NavigateToTask::notStarted =======\n" << std::endl;
        std::cout << "\n==========================================\n" << std::endl;
    }
}

/*
    The navigate to task is not complete until the robot has reached the navigation endpoint
    AND, if required, oriented itself at the endpoint at the desired orientation. 
*/
void NavigateToTask::inProgress(std::shared_ptr<Map> map, 
                                std::shared_ptr<Navigator> navigator, 
                                RobotState& nextRobotState)
{
    double destX1 = map->getNextDestinationXY().getX(); 
    double destY1 = map->getNextDestinationXY().getY();
    double robotX = map->RobotX();
    double robotY = map->RobotY();
    double endpointDesiredOrientation = map->getDestinationOrientation();
    RobotPoseToWaypoint isRobotOnPath = navigator->isRobotOnPath(map);

    // get angle required for robot to rotate until it reaches the angle required at endpoint
    navigator->getRobotToEndpointSlopeAngle(map, endpointDesiredOrientation);
    
    switch(isRobotOnPath) {
        case NONE:
            // decide whether to move forward or backward depending on distance to endpoint
            break;
        case NEAR:
            // fix endpoint if the task requires it
            if(isEndpointOrientationRequired == false) {
                setStatus(COMPLETE);
                nextRobotState = STOP;
                isRobotAtEndpoint = true;
            }
            else if(isEndpointOrientationRequired) {
                setStatus(INPROGRESS);
                EndpointPoseCorrection(map, navigator, nextRobotState);
            }

            break;
        case ON_PATH:
            setStatus(INPROGRESS);
            nextRobotState = MOVE_FORWARD;
            //robotState = robot->getTravelDirection();
            break;
        case OFF_PATH:
            setStatus(SUSPENDED);
            nextRobotState = STOP;
            break;
    }

    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "======= NavigateToTask::inProgress =======\n";
        std::cout << "robot pose relative to waypoint: " 
                    << printRobotPoseToWaypoint(isRobotOnPath) << "\n";
        std::cout << "==================================\n" << std::endl;
    }

}

void NavigateToTask::suspended(std::shared_ptr<Map> map, 
                               std::shared_ptr<Navigator> navigator, 
                               RobotState& nextRobotState, TaskType& nextTaskType) 
{
    // refer to the member data containing next-task-type
    nextTaskType = PATHCORRECTION;
    nextTaskType = POSECORRECTION;
    nextRobotState = STOP;
}

/*
    When a robot has completed traveling to its endpoint, it then needs to 
    correct its orientation so that it matches the orientation required
    by the endpoint pose.
*/
void NavigateToTask::complete(std::shared_ptr<Map> map, 
                              std::shared_ptr<Navigator> navigator, 
                              RobotState& nextRobotState, TaskType& nextTaskType) 
{
    // What used to be the pose correction task is now embedded within this
    // class as the EnpointPoseCorrection function which is called when the
    // robot has reached the destination but the task requires the robot to
    // have a particular pose (orientation) at the endpoint. Instead of creating
    // a new pose correction task it is more efficient to create a function that 
    // will execute at the next state machine 'tick'.
    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "\n======= NavigateToTask::complete =======\n" << std::endl;
    }
}

void NavigateToTask::EndpointPoseCorrection(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = navigator->isRobotOriented(map, endpointDesiredOrientation);
        
    // assign robot new state depending on its orientation relative to waypoint
    switch(robotOrientationAtEndpoint) {
        case ORIENTED:
            setStatus(COMPLETE);
            nextRobotState = STOP;
            break;
        case OFF_TO_RIGHT:
            setStatus(INPROGRESS);
            nextRobotState = ROTATE_CCW;
            break;
        case OFF_TO_LEFT:
            setStatus(INPROGRESS);
            nextRobotState = ROTATE_CW;
            break;
    }
}