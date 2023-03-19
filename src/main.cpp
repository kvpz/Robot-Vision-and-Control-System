#include <boost/asio.hpp>
#include <iostream>
#include <stack>
#include <vector>
#include <utility>
#include "robot.hpp"
//#include "task.hpp"
#include "taskmanager.hpp"
#include "example-utils.hpp"

#define DEBUG_MAIN false

double _180_over_PI = 180.0 / M_PI;
double _PI_over_180 = M_PI / 180.0;

// xy offset from (0,0) xy base values initiated at startup of T265
double x_offset = 120.0;
double y_offset = 30.0;

//std::stack<Task> task_queue;

//TaskManager taskManager;

int main() try
{
    Robot* robot = new Robot();
    robot->setCurrentXY(x_offset, y_offset); // x,y are front of robot (camera location)
    
    // load tasks from JSON file
    robot->getTaskManager()->importTasksFromJSON();

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

            double current_angle_quaternion = pose_data.rotation.y;
          
            double w = pose_data.rotation.w;
            double x = -1.0 * pose_data.rotation.z;
            double y = pose_data.rotation.x;
            double z = -1.0 * pose_data.rotation.y;
            double yaw = atan((2.0 * (w*z + x*y)) /  (w*w + x*x - y*y - z*z)) * _180_over_PI;
            robot->setOrientation(yaw_to_degrees(yaw, pose_data.rotation.y));
            
            double current_x = (pose_data.translation.x) * 100.0 + x_offset;
            double current_y;
            if(pose_data.translation.z < 0.0)
              current_y = std::fabs(pose_data.translation.z) * 100.0 + y_offset;
            else
              current_y = y_offset - ((pose_data.translation.z) * 100.0);

            int quadrant = quadrant_identifier(yaw, current_angle_quaternion);
            if(quadrant == 1) {
              current_x = current_x - 15.0 * std::cos((90.0 - yaw) * _PI_over_180);
              current_y = current_y - 15.0 * std::sin((90.0 - yaw) * _PI_over_180);
            }
            else if(quadrant == 2) {
              current_x = current_x + 15.0 * std::cos((90.0 + yaw) * _PI_over_180);
              current_y = current_y - 15.0 * std::sin((90.0 + yaw) * _PI_over_180);
            }
            else if(quadrant == 3) {
              current_x = current_x + 15.0 * std::cos((90.0 - yaw) * _PI_over_180);
              current_y = current_y + 15.0 * std::sin((90.0 - yaw) * _PI_over_180);
            }
            else {
              current_x = current_x - 15.0 * std::cos((90.0 + yaw) * _PI_over_180);
              current_y = current_y + 15.0 * std::sin((90.0 + yaw) * _PI_over_180);
            }

            robot->setCurrentXY(current_x, current_y);
        }
    };

    // start T265 data pipeline
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    // allow time for camera to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));			  

    while(1) {
      if(!robot->hasTasks())
        break;

      robot->executeCurrentTask();

      std::this_thread::sleep_for(std::chrono::milliseconds(10));	

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

/*
  Notes:
  Testing Results #1:
  sampling: 400 ms
  angle to destination tolerance: 4 degrees


*/  

      /*
      if(DEBUG_MAIN) {
        std::cout << "=========== Main Loop ============\n";
        std::cout << "task stack size: " << task_queue.size() << "\n";
        std::cout << "current task type: " << taskTypeToString(currentTask.getTaskType()) << "\n";
        std::cout << "==================================" << std::endl;
        robot.printStatus();
      }
      */