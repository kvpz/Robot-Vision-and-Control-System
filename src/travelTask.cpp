#include "travelTask.hpp"

TravelTask::TravelTask(){}

// Task subtasks

void TravelTask::notStarted(Robot* robot) //override
{
    // At this point the robot should be told whether it should travel 
    // forward, backward, left, or right depending on its distance
    // to the endpoint

    if(distance(robot->getX(), destination.getX(), destination.getY(), robot->getY()) < 70.0 && robot->getAngleToDestination() > 130.0)
        robot->setTravelDirection(MOVE_BACKWARD); //travelDirection = MOVE_BACKWARD;
    else
        robot->setTravelDirection(MOVE_FORWARD);//travelDirection = MOVE_FORWARD;

    setStatus(INPROGRESS);
}

void TravelTask::inProgress(Robot* robot) //override
{
    double destX1 = destination.getX();
    double destY1 = destination.getY();
    double robotX1 = robot->getX();
    double robotY1 = robot->getY();

    RobotPoseToWaypoint rposetoway;
    rposetoway = robot->isRobotOnPath(robotX1, robotY1, destX1, destY1);

    robot->getRobotAngleToPoseOrientation(getEndpointOrientation());

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
        robotState = robot->getTravelDirection();
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
