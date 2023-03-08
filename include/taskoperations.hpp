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

            //RobotPoseToWaypoint rposetoway = robot.robotPositionRelativeToWaypoint(robotX, robotY, destX, destY);
            RobotPoseToWaypoint rposetoway = robot.isRobotOnPath(robotX, robotY, destX, destY);

            if(DEBUG_TSKOPS) {
                std::cout << "(travel_task_updater) robot pose relative to waypoint: " 
                << printRobotPoseToWaypoint(rposetoway) << std::endl;
            }
            
            // assign robot a task depending on where it is relative to waypoint
            switch(rposetoway) {
            case NEAR:
                task.setStatus(COMPLETE);
                robotState = STOP;
                break;
            case BEFORE_LEFT:
                task.setStatus(SUSPENDED);
                robotState = ROTATE_CW;
                break;
            case BEFORE_RIGHT:
                task.setStatus(SUSPENDED);
                robotState = ROTATE_CCW;
                break;
            case AFTER_LEFT:
            case AFTER_RIGHT:
            case ON_PATH:
                //robot.setState(MOVE_BACKWARD);
                robotState = MOVE_FORWARD;
            case OFF_PATH:
                task.setStatus(SUSPENDED);
                robotState = STOP;
                break;
            }
            

            printTaskInfo(task);
        }

        static void correctpath_task_updater(Robot& robot, Task& task, RobotState& robotState)
        {
            double destX = task.getDestination().getX();
            double destY = task.getDestination().getY();
            double robotX = robot.getX();
            double robotY = robot.getY();
            
            RobotPoseToWaypoint rposetoway = robot.isRobotOnPath(robotX, robotY, destX, destY);

            printTaskInfo(task);
        }

    private:
        static void printTaskInfo(Task& task)
        {
            // print status of this type of task
            switch(task.getTaskType()) {
                case TRAVEL:
                    std::cout << "\n====== Travel Task Updater ======\n";
                    break;
                case CORRECTPATH:
                    std::cout << "\n====== CorrectPath Task Updater ======\n";
                    break;  
            }

            std::cout << "Task destination (X,Y): (" << task.getDestination().getX() 
                    << ", " << task.getDestination().getY() << ")\n";
            std::cout << "Task status: " << task.getStatus() << "\n";
            std::cout << "Task name: " << task.getName() << "\n";
            std::cout << "=================================\n";
            std::cout << std::endl;
        }
    };

}

        /*
        double rotation_correction(double degrees_actually_rotated, double target, bool cw)
        {
            double total_rotation = 0.0;
            pre_action_angle = yaw_to_degrees(yaw, current_angle_quaternion);//convert_quaternions_to_degrees(current_angle_quaternion);
            if(degrees_actually_rotated < target) {
                //std::this_thread::sleep_for(std::chrono::milliseconds(200));
                if(cw)
                    rotate_CW(std::fabs(90.0 - degrees_actually_rotated), ROBOT_360_ROTATE_TIME_MILLISEC);
                else
                    rotate_CCW(std::fabs(90.0 - degrees_actually_rotated), ROBOT_360_ROTATE_TIME_MILLISEC);
            }
            else {
                if(cw)
                    rotate_CW(std::fabs(90.0 - degrees_actually_rotated), ROBOT_360_ROTATE_TIME_MILLISEC);
                else
                    rotate_CCW(std::fabs(90.0 - degrees_actually_rotated), ROBOT_360_ROTATE_TIME_MILLISEC);
            }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                post_action_angle = yaw_to_degrees(yaw, current_angle_quaternion); //convert_quaternions_to_degrees(current_angle_quaternion);
                
                total_rotation = std::fabs(post_action_angle - pre_action_angle);
                std::cout << "rotation_correction: " << total_rotation << " (total rotation)" << std::endl;
                return total_rotation;
        }

        void rotation_correction_until(double degrees_actually_rotated, double degrees_to_rotate, bool cw)
        {  
            while(approximately(degrees_actually_rotated, degrees_to_rotate, 1.2, true) == false) {
                pre_action_angle = yaw_to_degrees(yaw, current_angle_quaternion); //convert_quaternions_to_degrees(current_angle_quaternion);
                double total_degrees_rotated = rotation_correction(degrees_actually_rotated, degrees_to_rotate, cw);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                post_action_angle = yaw_to_degrees(yaw, current_angle_quaternion); //convert_quaternions_to_degrees(current_angle_quaternion);
                degrees_actually_rotated += std::fabs(post_action_angle - pre_action_angle);
                
                // if overshoot, then break out of while loop
                if(degrees_actually_rotated > degrees_to_rotate){
                    break;
                }
            }
        }
        */
#endif