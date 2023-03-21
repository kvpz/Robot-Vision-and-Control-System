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

void NavigateToTask::notStarted(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState)
{
    // At this point the robot should be told whether it should travel 
    // forward, backward, left, or right depending on its distance
    // to the endpoint
    double robotDistanceToEndpoint = distance(map->RobotX(), destination.getX(), destination.getY(), map->getRobotCurrentYCoordinatePoint());
    double robot_orientation_minus_destination = navigator->getAngleToDestination();

    if(robotDistanceToEndpoint < 70.0 && robot_orientation_minus_destination > 130.0) //robot->getAngleToDestination() > 130.0)
        nextRobotState = MOVE_BACKWARD;
        //robot->setTravelDirection(MOVE_BACKWARD); //travelDirection = MOVE_BACKWARD;
    else
        nextRobotState = MOVE_FORWARD;
        //robot->setTravelDirection(MOVE_FORWARD);//travelDirection = MOVE_FORWARD;

    setStatus(INPROGRESS);

    if(DEBUG_NAVIGATETOTASK) {
        std::cout << "\n====== NavigateToTask::notStarted =======\n" << std::endl;
    }
}

/*
    The navigate to task is not complete until the robot has reached the navigation endpoint
    AND, if required, oriented itself at the endpoint at the desired orientation. 
*/
void NavigateToTask::inProgress(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState)
{
    double destX1 = destination.getX(); 
    double destY1 = destination.getY();
    double robotX = map->RobotX();
    double robotY = map->RobotY();
    double endpointDesiredOrientation = map->dest
    RobotPoseToWaypoint isRobotOnPath = navigator->isRobotOnPath(std::move(map), robotX, robotY, destX1, destY1);

    // get angle required for robot to rotate until it reaches the angle required at endpoint
    navigator->getRobotToEndpointSlopeAngle(std::move(map), endpointDesiredOrientation);
    
    switch(isRobotOnPath) {
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
                EndpointPoseCorrection(std::move(map), std::move(navigator), nextRobotState);
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
                << printRobotPoseToWaypoint(isRobotOnPath) << std::endl;
    }

}

void NavigateToTask::suspended(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) 
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
void NavigateToTask::complete(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) 
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

void NavigateToTask::EndpointPoseCorrection(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = navigator->isRobotOriented(std::move(map), endpointDesiredOrientation);
        
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