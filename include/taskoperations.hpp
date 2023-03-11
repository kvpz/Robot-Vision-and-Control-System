#ifndef TASKOPERATIONS_HPP
#define TASKOPERATIONS_HPP
#include "robot.hpp"
#include "task.hpp"
#define DEBUG_TSKOPS true

namespace ROBOTASKS 
{
    class TaskOperations 
    {
    public:

        static void travel_task_updater(Robot& robot, Task& task, RobotState& robotState)
        {
            double destX = task.getDestination().getX();
            double destY = task.getDestination().getY();
            double robotX = robot.getX();
            double robotY = robot.getY();
            //double angleToDestTolerance = 1.0;

            //RobotPoseToWaypoint rposetoway = robot.robotPositionRelativeToWaypoint(robotX, robotY, destX, destY);
            RobotPoseToWaypoint rposetoway = robot.isRobotOnPath(robotX, robotY, destX, destY);

            if(DEBUG_TSKOPS) {
                std::cout << "(travel_task_updater) robot pose relative to waypoint: " 
                << printRobotPoseToWaypoint(rposetoway) << std::endl;
            }
            
            // assign robot a task depending on orientation relative to waypoint
            switch(rposetoway) {
            case NEAR:
                task.setStatus(COMPLETE);
                robotState = STOP;
                break;
            case ON_PATH:
                task.setStatus(INPROGRESS);
                robotState = MOVE_FORWARD;
                break;
            case OFF_PATH:
                task.setStatus(SUSPENDED);
                robotState = STOP;
                break;
            case AFTER_LEFT:
            case AFTER_RIGHT:
                break;
            }

            Task::printTaskInfo(task);
        }

        static void correctpath_task_updater(Robot& robot, Task& task, RobotState& robotState)
        {
            double destX = task.getDestination().getX();
            double destY = task.getDestination().getY();
            double robotX = robot.getX();
            double robotY = robot.getY();
            double angleToDestTolerance = 10.0;
            RobotPoseToWaypoint rposetoway = robot.isRobotOnPath(robotX, robotY, destX, destY);

            // assign robot a task depending on orientation relative to waypoint
            switch(rposetoway) {
            case NEAR:
                task.setStatus(COMPLETE);
                robotState = STOP;
                break;
            case ON_PATH:
                task.setStatus(COMPLETE);
                //robotState = MOVE_FORWARD;
		robotState = STOP;
                break;
            case OFF_PATH:
                task.setStatus(INPROGRESS);
                if(robot.angleToDestination() > angleToDestTolerance)
                    robotState = ROTATE_CCW;
                else
                    robotState = ROTATE_CW;
                break;
            case AFTER_LEFT:
            case AFTER_RIGHT:
                break;
            }

            Task::printTaskInfo(task);
        }
    };

}

#endif
