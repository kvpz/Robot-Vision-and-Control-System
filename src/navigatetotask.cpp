#include "navigatetotask.hpp"

NavigateToTask::NavigateToTask(double endpointOrientation, bool endpointOrientationRequirement) 
    : isRobotAtEndpoint(false), Task(NAVIGATETO)
{
    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "\n======= NavigateToTask::NavigateToTask =======\n";
        std::cout << "isEndpointOrientationRequired: " << endpointOrientationRequirement << "\n";
        std::cout << "endpointOrientation: " << endpointOrientation << "\n";
        std::cout << "==============================================\n" << std::endl;
    }
    isEndpointOrientationRequired = endpointOrientationRequirement;
    endpointDesiredOrientation = endpointOrientation;
}

void NavigateToTask::notStarted(Map* map, Navigator* navigator, RobotState& nextRobotState)
{
    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "\n====== NavigateToTask::notStarted =======\n" << std::endl;
    }
    // At this point the robot should be told whether it should travel 
    // forward, backward, left, or right depending on its distance
    // to the endpoint
    if(distance(map->getRobotCurrentXCoordinatePoint(), destination.getX(), destination.getY(), map->getRobotCurrentYCoordinatePoint()) < 70.0 && 
        navigator->getAngleToDestination() > 130.0) //robot->getAngleToDestination() > 130.0)
        nextRobotState = MOVE_BACKWARD;
        //robot->setTravelDirection(MOVE_BACKWARD); //travelDirection = MOVE_BACKWARD;
    else
        nextRobotState = MOVE_FORWARD;
        //robot->setTravelDirection(MOVE_FORWARD);//travelDirection = MOVE_FORWARD;

    setStatus(INPROGRESS);
}

/*
    The navigate to task is not complete until the robot has reached the navigation endpoint
    AND, if required, oriented itself at the endpoint at the desired orientation. 
*/
void NavigateToTask::inProgress(Map* map, Navigator* navigator, RobotState& nextRobotState)
{
    double destX1 = destination.getX(); 
    double destY1 = destination.getY();

    RobotPoseToWaypoint robotPoseRelativeToWaypoint;
    robotPoseRelativeToWaypoint = navigator->isRobotOnPath(map, map->getRobotCurrentXCoordinatePoint(), map->getRobotCurrentYCoordinatePoint(), destX1, destY1);

    // get angle required for robot to rotate until it reaches the angle required at endpoint
    navigator->getRobotToEndpointSlopeAngle(map, endpointDesiredOrientation);
    
    switch(robotPoseRelativeToWaypoint) {
        case NONE:
            // decide whether to move forward or backward depending on distance to endpoint
            break;
        case NEAR:
            // set to complete if orientation at endpoint does not matter
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
            //robotState = robot->getTravelDirection();
            break;
        case OFF_PATH:
            setStatus(SUSPENDED);
            nextRobotState = STOP;
            break;
    }

    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "======= NavigateToTask::inProgress =======" << std::endl;
        std::cout << "(navigateto_task_updater) robot pose relative to waypoint: " 
                << printRobotPoseToWaypoint(robotPoseRelativeToWaypoint) << std::endl;
    }

}

void NavigateToTask::EndpointPoseCorrection(Map* map, Navigator* navigator, RobotState& nextRobotState) 
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

void NavigateToTask::suspended(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType) 
{
    // travelTaskSuspendedState(task); definition below
    // if new task assumed to be CORRECTPATH
    //Task newTask(PATHCORRECTION);
    //newTask.setEndpoint(destination.getX(), destination.getY(), getEndpointDesiredOrientation());
    //task_queue.push(newTask);
    nextTaskType = PATHCORRECTION;
    nextRobotState = STOP;
}

/*
    When a robot has completed traveling to its endpoint, it then needs to 
    correct its orientation so that it matches the orientation required
    by the endpoint pose.
*/
void NavigateToTask::complete(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType) 
{
    //task_queue.pop();
    // travelTaskCompleteState(task) definition lines below
    //Task newTask(POSECORRECTION);
    //newTask.setEndpoint(destination.getX(), destination.getY(), getEndpointDesiredOrientation());
    //task_queue.push(newTask);
    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "\n======= NavigateToTask::complete =======\n" << std::endl;
    }
}
