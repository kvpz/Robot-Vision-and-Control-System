// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include "includes.hpp"
#include <boost/asio.hpp>
#include <stack>
#include "robot.hpp"
#include "task.hpp"
#include "taskoperations.hpp"
using namespace ROBOTASKS;

#define DEBUG true
double x_offset = 120.0;
double y_offset = 30.0;

double current_angle_quaternion = 0.0;

double current_x = 0.0;
double current_y = 0.0;
double pre_action_angle = 0.0;
double post_action_angle = 0.0;
double yaw = 0.0;

std::stack<Task> taskStack;

void travelTaskSuspendedState(Task& task) 
{
  // if new task assumed to be CORRECTPATH
  Task newTask(CORRECTPATH, "correctpath");
  newTask.setDestination(task.getDestination().getX(),
    task.getDestination().getY());
  taskStack.push(newTask);
}

void travelTaskManager(Task& task, Robot& robot, RobotState& nextRobotState)
{
  switch(task.getStatus()) {
    case NOTSTARTED:
      task.setStatus(INPROGRESS);
      break;  
    case INPROGRESS:
      ROBOTASKS::TaskOperations::travel_task_updater(robot, task, nextRobotState);
      break;
    case SUSPENDED:
      travelTaskSuspendedState(task);
      break;
    case COMPLETE:
      taskStack.pop();
      break;
  }
}

int main() try
{
  Robot robot;
  robot.setCurrentXY(120, 30); // x,y are front of robot (camera location)

  // second task is rotate ccw
  Task task2(TRAVEL, "task two");
  task2.setDestination(30, 25);
  task2.setStatus(NOTSTARTED);
  task2.setDesiredRobotYawPose(180);
  taskStack.push(task2);
  //robot.setTask(task2, "task two");

  // first task is move forward a bit
  Task task1(TRAVEL, "task one");
  task1.setDestination(120, 40);
  task1.setStatus(INPROGRESS);
  taskStack.push(task1);
  //robot.setTask(task, "task one");


  
  // Setup T265 connection
    std::string serial_t265_str;
    
    if (!device_with_streams({ RS2_STREAM_POSE }, serial_t265_str))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    // The pipeline is used to simplify the user interaction with the device and CV modules. 
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial_t265_str.empty())
        cfg.enable_device(serial_t265_str);
    // Add pose stream. The pose data packed as float arrays containing translation vector
    // rotation quaternion and prediction velocities and acceleration vectors
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    std::mutex mutex;
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
            rs2_pose pose_data = fp.get_pose_data();

            current_angle_quaternion = pose_data.rotation.y;

            current_x = pose_data.translation.x * 100.0 + x_offset;

            if(pose_data.translation.z < 0.0)
              current_y = std::fabs(pose_data.translation.z) * 100.0 + y_offset;
            else
              current_y = y_offset - (pose_data.translation.z * 100.0);
            
            robot.setCurrentXY(current_x, current_y);

            double w = pose_data.rotation.w;
            double x = -1.0 * pose_data.rotation.z;
            double y = pose_data.rotation.x;
            double z = -1.0 * pose_data.rotation.y;
            yaw = atan((2.0 * (w*z + x*y)) /  (w*w + x*x - y*y - z*z)) * (180.0 / M_PI);
            robot.setOrientation(yaw_to_degrees(yaw, pose_data.rotation.y));
        }
    };

    // start T265 data pipeline
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    // allow time for camera to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));			  

    while(1) {
      if(taskStack.empty())
        break;

      Task& currentTask = taskStack.top();
      RobotState nextRobotState;

      if(currentTask.getTaskType() == TRAVEL) {
        travelTaskManager(currentTask, robot, nextRobotState);
      }
      else if(currentTask.getTaskType() == CORRECTPATH) {
        ROBOTASKS::TaskOperations::correctpath_task_updater(robot, currentTask, nextRobotState);
      }   

      // change robot behavior if a new state assigned by task scheduler
      if(robot.getState() != nextRobotState) {
        robot.setState(nextRobotState);
        robot.run(currentTask);
      }

      // robot status (DEBUG)
      robot.printStatus();

      //print_info();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));			  
    }
	
    
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

// compare general task requirement to robot's current state
// (1) if robot is not near destination, keep traveling to location
// (2) check if robot has dropped chips if [RED/GREEN]CHIPDROP task
// (3) check if robot has recycled objects if RECYCLING task

// check status of travel task and decide on robot state
	  
// robot has already been assigned a state

// TRAVEL Subtasks
// (1) if robot potentially stuck, undo travel
// (2) if robot not following path (subtask), correct path
// (2.1) set state STOP
// (2.2) set state rotate_CW if current angle larger than expected
// (2.3) set state rotate_CCW if current angle smaller than expected
// path following is checked every t time or x distance. Path is not being
// followed if orientation angle is much different than the previous reading.
// (subtask)

// if CHIPDROP task
// CHIPDROP Subtasks
// (1) if robot not in correct orientation, orient robot
// (2) if robot has rotated and is not near position of attraction, orient robot
// (3) 

// if RECYCLE task
// Verify the pincers are open. 

