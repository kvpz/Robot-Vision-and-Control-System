#include "navigatetotask.hpp"

NavigateToTask::NavigateToTask()
:  destinationOrientationTolerance(ORIENTATION_RANGE_TOLERANCE), 
    Task(TaskType::NAVIGATETO, NAVIGATETOTTASK_PRIORITY) 
{}

NavigateToTask::NavigateToTask(double endpointOrientation, 
                               bool endpointOrientationRequirement) 
    : isRobotAtEndpoint(false), 
    Task(NAVIGATETO, NAVIGATETOTTASK_PRIORITY), 
    destinationOrientationTolerance(ORIENTATION_RANGE_TOLERANCE)
{
    isEndpointOrientationRequired = endpointOrientationRequirement;
    endpointDesiredOrientation = endpointOrientation;

    if(DEBUG_NAVIGATETOTASK) {
        printTaskInfo("NavigateToTask::NavigateToTask");        
    }
}

NavigateToTask::NavigateToTask(XYPoint xy, 
                               double endpointOrientation, 
                               bool endpointOrientationRequired, TravelDirection travelDir)
    : isRobotAtEndpoint(false), Task(NAVIGATETO, NAVIGATETOTTASK_PRIORITY), 
    destinationOrientationTolerance(ORIENTATION_RANGE_TOLERANCE)
{    
    endpoint.setX(xy.getX());
    endpoint.setY(xy.getY());
    endpointDesiredOrientation = endpointOrientation;
    isEndpointOrientationRequired = endpointOrientationRequired;
    travelDirection = travelDir;
}


/*
    This function should initialize the map because the task 
    contains information about the endpoint.
*/
void NavigateToTask::notStarted(std::shared_ptr<Map> map, 
                                std::shared_ptr<Navigator> navigator, RobotState& nextRobotState)
{
    // set next endpoint to navigate to in map
    map->setDestinationXY(endpoint.getX(), endpoint.getY());
    if(isEndpointOrientationRequired)
        map->setDestinationDesiredOrientation(endpointDesiredOrientation);

    // At this point the robot should be told whether it should travel 
    // forward, backward, left, or right depending on its distance to the endpoint and current location
    double robotDistanceToEndpoint = distance(map->getRobotCurrentLocation(), map->getNextDestinationXY());
    //double robot_orientation_minus_destination = navigator->robotAngularDistanceToOrientation(map); //- map->getDestinationOrientation(); //navigator->getAngleToDestination();

    // give the navigator information about robot's required travel direction
    navigator->setTravelDirection(travelDirection);
/*
    if(robotDistanceToEndpoint < 70.0 && 
       navigator->robotAngularDistanceToEndpoint(map, true) > 130.0)
        // move robot backward to a point if travel distance is short
        // and the point is behind the robot
        nextRobotState = MOVE_BACKWARD;
    else
        nextRobotState = MOVE_FORWARD;
*/
    // update task status
    status = TaskStatus::INPROGRESS;

    /*
        TODO:
        (1) implement optimal travel direction
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
    switch(navigator->isRobotOnPath(map)) {
        case NEAR:
            // fix endpoint if the task requires it
            if(isEndpointOrientationRequired == false) {
                status = TaskStatus::COMPLETE;
                nextRobotState = STOP;
                isRobotAtEndpoint = true;
            }
            else if(isEndpointOrientationRequired && 
                    !approximately(map->getRobotOrientation(), map->getDestinationOrientation(), destinationOrientationTolerance)) {
                status = TaskStatus::SUSPENDED;
                newTaskRequest = POSECORRECTION;
                nextRobotState = STOP;
            }
            else {
                status = TaskStatus::COMPLETE;
                nextRobotState = STOP;
            }
            break;

        case ON_PATH:
            status = TaskStatus::INPROGRESS;
            if(travelDirection == TravelDirection::forward)
                nextRobotState = MOVE_FORWARD;
            else if(travelDirection == TravelDirection::backward)
                nextRobotState = MOVE_BACKWARD;
            break;

        case OFF_PATH:
            status = TaskStatus::SUSPENDED;
            newTaskRequest = PATHCORRECTION;
            nextRobotState = STOP;
            break;
    }

    if(DEBUG_NAVIGATETOTASK) {
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
    nextTaskType = newTaskRequest;
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

    // clear the destination from the map
    map->setIsEndpointOrientationRequired(false);
    map->setDestinationXY(-1.0, -1.0);

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
