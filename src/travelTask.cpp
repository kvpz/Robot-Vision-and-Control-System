#include "travelTask.hpp"
//#include "utility.hpp"

TravelTask::TravelTask(){}

// Task subtasks

void TravelTask::notStarted(Map* map, Navigator* navigator, RobotState& robotState)
{
    // At this point the robot should be told whether it should travel 
    // forward, backward, left, or right depending on its distance
    // to the endpoint

    if(distance(map->getRobotCurrentXCoordinatePoint(), destination.getX(), destination.getY(), map->getRobotCurrentYCoordinatePoint()) < 70.0 && 
        navigator->getAngleToDestination() > 130.0) //robot->getAngleToDestination() > 130.0)
        robotState = MOVE_BACKWARD;
        //robot->setTravelDirection(MOVE_BACKWARD); //travelDirection = MOVE_BACKWARD;
    else
        robotState = MOVE_FORWARD;
        //robot->setTravelDirection(MOVE_FORWARD);//travelDirection = MOVE_FORWARD;

    setStatus(INPROGRESS);
}

//void TravelTask::inProgress(double rX, double rY, double rOrientation, double rAngleToDestination, RobotState& robotState)
void TravelTask::inProgress(Map* map, Navigator* navigator, RobotState& robotState)
{
    double destX1 = destination.getX(); 
    double destY1 = destination.getY();

    RobotPoseToWaypoint rposetoway;
    rposetoway = navigator->isRobotOnPath(map->getRobotCurrentXCoordinatePoint(), map->getRobotCurrentYCoordinatePoint(), destX1, destY1);

    // get angle required for robot to rotate until it reaches the angle required at endpoint
    navigator->getRobotAngleToPoseOrientation(map, getEndpointOrientation());

    if(DEBUG_TRAVELTASK) {
        std::cout << "(travel_task_updater) robot pose relative to waypoint: " 
        << printRobotPoseToWaypoint(rposetoway) << std::endl;
    }
    
    // assign robot a task depending on orientation relative to waypoint
    switch(rposetoway) {
    case NONE:
        // decide whether to move forward or backward depending on distance to endpoint
        break;
    case NEAR:
        setStatus(COMPLETE);
        robotState = STOP;
        //robot.setIsNearEndpoint(true);
        break;
    case ON_PATH:
        setStatus(INPROGRESS);
        //robotState = robot->getTravelDirection();
        break;
    case OFF_PATH:
        setStatus(SUSPENDED);
        robotState = STOP;
        break;
    }
}

void TravelTask::suspended() //override
{
    // travelTaskSuspendedState(task); definition below
    // if new task assumed to be CORRECTPATH
    //Task newTask(CORRECTPATH);
    //newTask.setEndpoint(destination.getX(), destination.getY(), getEndpointOrientation());
    //task_queue.push(newTask);
}

/*
    When a robot has completed traveling to its endpoint, it then needs to 
    correct its orientation so that it matches the orientation required
    by the endpoint pose.
*/
void TravelTask::complete() //override
{
    //task_queue.pop();
    // travelTaskCompleteState(task) definition lines below
    //Task newTask(ORIENT);
    //newTask.setEndpoint(destination.getX(), destination.getY(), getEndpointOrientation());
    //task_queue.push(newTask);
}
