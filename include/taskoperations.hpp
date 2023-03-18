#ifndef TASKOPERATIONS_HPP
#define TASKOPERATIONS_HPP
#include "robot.hpp"
#include "task.hpp"
#define DEBUG_TSKOPS true

static bool correcting_position = false;
static bool correcting_orientation = false;

namespace ROBOTASKS 
{
    double angleToDestTolerance = 10.0;

    class TaskOperations 
    {
    private: 
      
    public:

        static void travel_task_updater(Robot& robot, Task& task, RobotState& robotState)
        {
            double destX1 = task.getDestination().getX();
            double destY1 = task.getDestination().getY();
            double robotX1 = robot.getX();
            double robotY1 = robot.getY();

            RobotPoseToWaypoint rposetoway;
            //RobotPoseToWaypoint rposetoway = robot.robotPositionRelativeToWaypoint(robotX, robotY, destX, destY);
            rposetoway = robot.isRobotOnPath(robotX1, robotY1, destX1, destY1);

	        robot.getRobotAngleToPoseOrientation(robot, task.getEndpointOrientation());
	    
            if(DEBUG_TSKOPS) {
                std::cout << "(travel_task_updater) robot pose relative to waypoint: " 
                << printRobotPoseToWaypoint(rposetoway) << std::endl;
            }
            
            // assign robot a task depending on orientation relative to waypoint
            switch(rposetoway) {
	        case NONE:
                // decide whether to move forward or backward depending on distance to endpoint
                break;
            case NEAR:
                task.setStatus(COMPLETE);
                robotState = STOP;
                //robot.setIsNearEndpoint(true);
                break;
            case ON_PATH:
                task.setStatus(INPROGRESS);
                robotState = robot.getTravelDirection();//MOVE_FORWARD;
                break;
            case OFF_PATH:
                task.setStatus(SUSPENDED);
                robotState = STOP;
                break;
            }

            /*
            // This is an attempt to get the robot oriented when reach endpoint
            // set robot pose at endpoint
            if(rposetoway == NEAR) {
                if(approximately(robot.getOrientation(), task.getEndpointOrientation(), 3.0, false)) {
                    robotState = STOP;
                    task.setStatus(COMPLETE);
                }
                else {
                    robotState = ROTATE_CW;
                }

            }
            */
            Task::printTaskInfo(task);
        }

        static void correctpath_task_updater(Robot& robot, Task& task, RobotState& robotState)
        {
            double destX2 = task.getDestination().getX();
            double destY2 = task.getDestination().getY();
            double robotX2 = robot.getX();
            double robotY2 = robot.getY();
            RobotPoseToWaypoint rposetoway = robot.isRobotOnPath(robotX2, robotY2, destX2, destY2);

            // assign robot a task depending on orientation relative to waypoint
            switch(rposetoway) {
            case NEAR:
                task.setStatus(COMPLETE);
                robotState = STOP;
                correcting_position = false;
                break;
            case ON_PATH:
                task.setStatus(COMPLETE);
                //robotState = MOVE_FORWARD;
		robotState = STOP;
                correcting_position = false;
                break;
            case OFF_PATH:
                task.setStatus(INPROGRESS);
                if(correcting_position == false) {
		    // decide whether to rotate CW or CCW
                    if(robot.getAngleToDestination() > angleToDestTolerance) 
                        robotState = ROTATE_CCW;
                    else
                        robotState = ROTATE_CW;
		    
                    correcting_position = true;
                }
                else {
                    correcting_position = true;
                }
		
                break;
            }

            Task::printTaskInfo(task);
        }

      static void orient_task_updater(Robot& robot, Task& task, RobotState& robotState)
      {
	RobotOrientationAtEndpoint robotOrientationAtEndpoint = robot.isRobotOriented(robot.getOrientation(), task.getEndpointOrientation());
	    
	// assign robot a task depending on orientation relative to waypoint
	switch(robotOrientationAtEndpoint) {
	case ORIENTED:
	  task.setStatus(COMPLETE);
	  robotState = STOP;
	  correcting_orientation = false;
	  break;
	case OFF_TO_RIGHT:
	  task.setStatus(INPROGRESS);
	  robotState = ROTATE_CCW;
	  correcting_orientation = true;
	  break;
	case OFF_TO_LEFT:
	  task.setStatus(INPROGRESS);
	  robotState = ROTATE_CW;
	  correcting_orientation = true;
	  break;
	}

	Task::printTaskInfo(task);
      }
      
      static void dropchip_task_updater(Robot& robot, Task& task, RobotState& robotState)
      {
	
      }
    };

}

#endif
