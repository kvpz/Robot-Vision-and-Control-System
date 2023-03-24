#include "navigatetotask.hpp"

NavigateToTask::NavigateToTask(double endpointOrientation, 
                               bool endpointOrientationRequirement) 
    : isRobotAtEndpoint(false), Task(NAVIGATETO)
{
    isEndpointOrientationRequired = endpointOrientationRequirement;
    endpointDesiredOrientation = endpointOrientation;
    destinationOrientationTolerance = 2.0;

    if(DEBUG_NAVIGATETOTASK) {
        printTaskInfo("NavigateToTask::NavigateToTask");        
    }
}

NavigateToTask::NavigateToTask(XYPoint xy, double endpointOrientation, bool endpointOrientationRequired)
    : isRobotAtEndpoint(false), Task(NAVIGATETO)
{    
    endpoint.setX(xy.getX());
    endpoint.setY(xy.getY());
    endpointDesiredOrientation = endpointOrientation;
    isEndpointOrientationRequired = endpointOrientationRequired;
    destinationOrientationTolerance = 2.0;  
}


/*
    This function should initialize the map because the task 
    contains information about the 
*/
void NavigateToTask::notStarted(std::shared_ptr<Map> map, 
                                std::shared_ptr<Navigator> navigator, RobotState& nextRobotState)
{
    // set next endpoint to navigate to in map
    map->setDestinationXY(endpoint.getX(), endpoint.getY());
    if(this->getIsEndpointOrientationRequired())
        map->setDestinationDesiredOrientation(this->getEndpointDesiredOrientation());

    // At this point the robot should be told whether it should travel 
    // forward, backward, left, or right depending on its distance to the endpoint and current location
    double robotDistanceToEndpoint = distance(map->getRobotCurrentLocation(), map->getNextDestinationXY());
    //double robot_orientation_minus_destination = navigator->robotAngularDistanceToOrientation(map); //- map->getDestinationOrientation(); //navigator->getAngleToDestination();

    if(robotDistanceToEndpoint < 70.0 && 
       navigator->robotAngularDistanceToOrientation(map) > 130.0)
        // move robot backward to a point if travel distance is short
        // and the point is behind the robot
        nextRobotState = MOVE_BACKWARD;
    else
        nextRobotState = MOVE_FORWARD;

    // update task status
    status = TaskStatus::INPROGRESS;

    /*
        TODO:
        (1) implement optimal travel direction
        (2)  
    */
    if(DEBUG_NAVIGATETOTASK) {
       printTaskInfo("NavigateToTask::NavigateToTask");        
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
                status = TaskStatus::COMPLETE;
                nextRobotState = STOP;
                isRobotAtEndpoint = true;
            }
            else if(isEndpointOrientationRequired && 
                    approximately(map->getRobotOrientation(), map->getDestinationOrientation(), destinationOrientationTolerance)) {
                status = TaskStatus::SUSPENDED;
                newTaskRequest = POSECORRECTION;
                //status = INPROGRESS);
                // move function endpoint pose correction function to pose correction task
                //EndpointPoseCorrection(map, navigator, nextRobotState);
            }
            else {
                status = TaskStatus::COMPLETE;
                nextRobotState = STOP;
            }

            break;
        case ON_PATH:
            status = TaskStatus::INPROGRESS;
            nextRobotState = MOVE_FORWARD;
            //robotState = robot->getTravelDirection();
            break;
        case OFF_PATH:
            status = TaskStatus::SUSPENDED;
            newTaskRequest = PATHCORRECTION;
            nextRobotState = STOP;
            break;
    }

    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "======= NavigateToTask::inProgress =======\n";
        std::cout << "robot pose relative to waypoint: " 
                    << printRobotPoseToWaypoint(isRobotOnPath) << "\n";
        std::cout << "==================================\n" << std::endl;
        printTaskInfo("NavigateToTask::inProgress");
    }

}

/*
    This is where the task makes the decision as to what type
    of task it needs to be completed, if any.
*/
void NavigateToTask::suspended(std::shared_ptr<Map> map, 
                               std::shared_ptr<Navigator> navigator, 
                               RobotState& nextRobotState, TaskType& nextTaskType) 
{
    // refer to the member data containing next-task-type
    nextTaskType = newTaskRequest;
    //nextTaskType = POSECORRECTION;
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
        printTaskInfo("NavigateToTask::complete");
    }
}

void NavigateToTask::printTaskInfo(std::string taskStateName)
{
        if(DEBUG_NAVIGATETOTASK) {
        std::cout << "\n====== " << taskStateName << " =======\n" << std::endl;
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "endpoint: " << endpoint << "\n";
        std::cout << "endpoint desired orientation: " << endpointDesiredOrientation << "\n";
        std::cout << "is robot at endpoint: " << isRobotAtEndpoint << "\n";
        std::cout << "is endpoint orientation required: " << isEndpointOrientationRequired << "\n";
        std::cout << "new task request: " << taskTypeToString(newTaskRequest) << "\n";
        std::cout << "destination orientation tolerance: " << destinationOrientationTolerance << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}

/*
void NavigateToTask::EndpointPoseCorrection(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = navigator->isRobotOriented(map, endpointDesiredOrientation);
        
    // assign robot new state depending on its orientation relative to waypoint
    switch(robotOrientationAtEndpoint) {
        case ORIENTED:
            status = COMPLETE);
            nextRobotState = STOP;
            break;
        case OFF_TO_RIGHT:
            status = INPROGRESS);
            nextRobotState = ROTATE_CCW;
            break;
        case OFF_TO_LEFT:
            status = INPROGRESS);
            nextRobotState = ROTATE_CW;
            break;
    }
}
*/