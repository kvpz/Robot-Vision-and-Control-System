// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include "../includes.hpp"

#define FEET_IN_METER 3.28084
double INCHES_IN_METER = 39.3701;
#define DEBUG false

double current_angle_quaternion = 0.0;
double current_x_rotation_quaternion = 0.0;
double current_y_rotation_quaternion = 0.0;
double current_z_rotation_quaternion = 0.0;
double current_w_rotation_quaternion = 0.0;

double current_x = 0.0;
double current_y = 0.0;
double pre_action_angle = 0.0;
double post_action_angle = 0.0;
double yaw = 0.0;

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

void print_info()
{
  std::cout << "current angle: " << yaw_to_degrees(yaw, current_angle_quaternion) << std::endl;
  std::cout << "current coordinate: ( " << current_x * 100 << ", " << current_y * 100 << " )" << std::endl;
  std::cout << "\n" << std::endl;
}

int main(int argc, char * argv[]) try
{
    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE }, serial))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    // The pipeline is used to simplify the user interaction with the device and CV modules. 
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
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

	    current_x = pose_data.translation.x;
	    current_y = pose_data.translation.z;
	    double currentXinches = pose_data.translation.x * INCHES_IN_METER;

	    double w = pose_data.rotation.w;
	    double x = -1.0 * pose_data.rotation.z;
	    double y = pose_data.rotation.x;
	    double z = -1.0 * pose_data.rotation.y;

	    current_w_rotation_quaternion = pose_data.rotation.w;
	    current_z_rotation_quaternion = pose_data.rotation.z;
	    current_x_rotation_quaternion = pose_data.rotation.x;
	    current_y_rotation_quaternion = pose_data.rotation.y;

	    yaw = atan((2.0 * (w*z + x*y)) /  (w*w + x*x - y*y - z*z)) * (180.0 / M_PI);
	}
    };

    // Start the pipeline streaming according to the configuration.
    // The pipeline captures the samples from the device and delivers them
    // to the frame callback. 
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    // go forward from initial position

    double distance_to_destination = 8.0;
    double degrees_to_rotate = 0.0;
    double degrees_actually_rotated = 0.0;

    int iterations = 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "going to destination" << std::endl;
    go_to_destination(122.0, 23.5, 62.0, 92.0, yaw_to_degrees(yaw, current_angle_quaternion));
    std::cout << "destination reached" << std::endl;    

    stop();
    
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
