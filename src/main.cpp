// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include "includes.hpp"
#include <boost/asio.hpp>
#include "robot.hpp"
#include "task.hpp"

#define DEBUG false

std::string serial_line_read;

double current_angle_quaternion = 0.0;

double current_x = 0.0;
double current_y = 0.0;
double pre_action_angle = 0.0;
double post_action_angle = 0.0;
double yaw = 0.0;

bool left_chips_deployed = false;
bool right_chips_deployed = false;

bool is_third_destination = false;

void serial_read_func()
{
    // setup async serial read
  boost::asio::io_context io_context;
  boost::asio::serial_port serial_read(io_context, "/dev/ttyACM0");
  
  // CONFIGURE NON-BLOCKING READS
  boost::asio::serial_port_base::flow_control flow_control(boost::asio::serial_port_base::flow_control::none);
  boost::asio::serial_port_base::parity parity(boost::asio::serial_port_base::parity::none);
  boost::asio::serial_port_base::stop_bits stop_bits(boost::asio::serial_port_base::stop_bits::one);
  boost::asio::serial_port_base::character_size char_size(8);
  
  boost::asio::streambuf buffer;
  //char data[1024];
  //std::string data;
  //boost::asio::async_read_until(serial_read, boost::asio::dynamic_buffer(data, sizeof(data)), '\n', //buffer, '\n',
  boost::asio::async_read_until(serial_read, buffer, '\n',
				//[data](const boost::system::error_code& error, size_t bytes_transferred) {
				[&](const boost::system::error_code& error, size_t bytes_transferred) {
				  if(!error) {
				    std::istream input_stream(&buffer);
				    std::string line;
				    //std::string line2;

				    std::getline(input_stream, line);
				    //serial_line_read = line;
				    std::cout << line;
				    
				    //std::getline(input_stream, line);
				    //serial_line_read = line;
				    
				    //std::cout << "in async_read_until callback" << std::endl;
				    //std::cout << "Received: " << line << std::endl;
				  }
				  else {
				    std::cerr << "Error: " << error.message() << std::endl;
				  }
				});
  io_context.run();
}

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

void first_destination()
{
  double distance_to_destination = 10.0;
  // move forward to first destination
  move_forward(distance_to_destination, ROBOT_SPEED_CM_PER_SEC);
  std::cout << "Distance traveled (odometry): ";
  //serial_read_func();
  std::cout << "Raw odometry data: ";
  //serial_read_func();
}

void first_destination_pose()
{
  double degrees_to_rotate = 90.0;
  double degrees_actually_rotated = 0.0;
  // rotate CCW 90 degrees
  pre_action_angle = yaw_to_degrees(yaw, current_angle_quaternion); //convert_quaternions_to_degrees(current_angle_quaternion);
  rotate_CCW(degrees_to_rotate, ROBOT_360_ROTATE_TIME_MILLISEC);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  post_action_angle = yaw_to_degrees(yaw, current_angle_quaternion); //convert_quaternions_to_degrees(current_angle_quaternion);
  degrees_actually_rotated = std::fabs(post_action_angle - pre_action_angle);
  
  std::cout << "post_action_angle (" << post_action_angle
	    << ")- pre_action_angle (" << pre_action_angle << ") : "
	    << degrees_actually_rotated << "\n" << std::endl;
  
  // verify robot rotated 90 degrees
  // if robot rotated less than 90 degrees, fix    
  // if robot rotated more than 90 degrees, fix
  rotation_correction_until(degrees_actually_rotated, degrees_to_rotate, false);
}

void second_destination()
{
  double distance_to_destination = 8.0;
  // go forward to bottom left corner of map
  distance_to_destination = 95.0;
  move_forward(distance_to_destination, ROBOT_SPEED_CM_PER_SEC);
  std::cout << "Distance traveled (odometry): ";
  //serial_read_func();
  std::cout << "Raw odometry data: ";
  //serial_read_func();
}

void second_destination_pose()
{
  double degrees_to_rotate = 90.0;
  double degrees_actually_rotated = 0.0;

  // rotate CW 90 degrees
  pre_action_angle = yaw_to_degrees(yaw, current_angle_quaternion); //convert_quaternions_to_degrees(current_angle_quaternion);

  if(is_third_destination)
    degrees_to_rotate = 90.0 - (90.0 - pre_action_angle);
  
  rotate_CW(degrees_to_rotate, ROBOT_360_ROTATE_TIME_MILLISEC);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  post_action_angle = yaw_to_degrees(yaw, current_angle_quaternion); //convert_quaternions_to_degrees(current_angle_quaternion);
  
  degrees_actually_rotated = std::fabs(pre_action_angle - post_action_angle);

  if(quadrant_identifier(post_action_angle) > quadrant_identifier(pre_action_angle)) {
    // example: quadrant 4
    //degrees_actually_rotated = std::fabs(post_action_angle - 180.0);
    degrees_actually_rotated = pre_action_angle + 360.0 - post_action_angle;
    std::cout << "post_action_angle (" << post_action_angle
	      << ")- pre_action_angle (" << pre_action_angle << ") : "
	      << degrees_actually_rotated << "\n" << std::endl;
  }
  
  
  // verify robot rotated 90 degrees
  rotation_correction_until(degrees_actually_rotated, degrees_to_rotate, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  send_command("R");
  std::this_thread::sleep_for(std::chrono::milliseconds((long)(16.0 / ROBOT_SPEED_CM_PER_SEC * 1000.0)));
  send_command("S");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  if(right_chips_deployed == false) {
    
    send_command("4");
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    send_command("3");
    right_chips_deployed = true;
  }
  else if(left_chips_deployed == false){
    
    send_command("1");
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    send_command("2");
    left_chips_deployed = true;
  }
}

void third_destination()
{
  double distance_to_destination = 8.0;
  // go forward to top left corner of map
  distance_to_destination = 70.0;
  move_forward(distance_to_destination, ROBOT_SPEED_CM_PER_SEC);
  std::cout << "Distance traveled (odometry): ";
  //serial_read_func();
  std::cout << "Raw odometry data: ";
  //serial_read_func();
}

void third_destination_pose()
{
  is_third_destination = true;
  second_destination_pose();
  is_third_destination = false;
}

void fourth_destination()
{
  double distance_to_destination = 10.0;
  // go forward to bottom left corner of map
  distance_to_destination = 160.0;
  move_forward(distance_to_destination, ROBOT_SPEED_CM_PER_SEC);
  std::cout << "Distance traveled (odometry): ";
  //serial_read_func();
  std::cout << "Raw odometry data: ";
  //serial_read_func();
}

void fourth_destination_pose()
{
  second_destination_pose();
}

void fifth_destination()
{
  third_destination();
}

void fifth_destination_pose()
{
  second_destination_pose();
}

void sixth_destination()
{
  fourth_destination();
}

void print_info()
{
  std::cout << "current angle: " << yaw_to_degrees(yaw, current_angle_quaternion) << std::endl;
  std::cout << "current coordinate: ( " << current_x * 100 << ", " << current_y * 100 << " )" << std::endl;
  std::cout << "\n" << std::endl;
}

void temp_test()
{
  move_forward(10.0, ROBOT_SPEED_CM_PER_SEC);
  std::cout << "Distance traveled (odometry): ";
  //serial_read_func();
  std::cout << "Raw odometry data: ";
  //serial_read_func();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  double degrees_actually_rotated = 0.0;
  
  while(1) {
    pre_action_angle = yaw_to_degrees(yaw, current_angle_quaternion);
    rotate_CW(45.0, ROBOT_360_ROTATE_TIME_MILLISEC);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    post_action_angle = yaw_to_degrees(yaw, current_angle_quaternion);
    degrees_actually_rotated = std::fabs(pre_action_angle - post_action_angle);
    //print_info();
  
  }
  
  
  rotate_CCW(360.0, ROBOT_360_ROTATE_TIME_MILLISEC);
}

void hardcoded_travel(int iterations)
{
    /*
  // forward backward warmup
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  move_forward(distance_to_destination, ROBOT_SPEED_CM_PER_SEC);
  std::cout << "Distance traveled (odometry): ";
  serial_read_func();
  std::cout << "Raw odometry data: ";
  serial_read_func();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  std::cout << "Distance traveled (odometry): " << serial_line_read << std::endl;
  print_info();
  
  move_backward(distance_to_destination, ROBOT_SPEED_CM_PER_SEC);
  std::cout << "Distance traveled (odometry): ";
  serial_read_func();
  std::cout << "Raw odometry data: ";
  serial_read_func();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  print_info();
  */
  std::cout << "first destination (iteration #" << iterations << ")" << std::endl;
  first_destination();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();

  first_destination_pose();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();

  if(iterations > 0) {
    std::cout << "second destination (iteration #" << iterations << ")" << std::endl;
    second_destination();
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
    print_info();
  }
  
  std::cout << "second destination (iteration #" << iterations << ")" << std::endl;
  second_destination();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();
  
  
  second_destination_pose();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();

  std::cout << "third destination (iteration #" << iterations << ")" << std::endl;
  third_destination();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();
  
  third_destination_pose();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();

  std::cout << "fourth destination (iteration #" << iterations << ")" << std::endl;
  fourth_destination();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();
  
  fourth_destination_pose();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();

  std::cout << "fifth destination (iteration #" << iterations << ")" << std::endl;
  fifth_destination();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();
  
  fifth_destination_pose();
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  print_info();
}

Waypoint w1(120, 35);

int main(int argc, char * argv[]) try
{
  Robot robot;
  robot.setCurrentXY(120, 30); // assuming x,y are front of robot (camera location)


  // first task is to drop chips
  Task task1;
  task1.setDestination(30, 15);  
  robot.setTask(task1);

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

            current_x = pose_data.translation.x;
            current_y = pose_data.translation.z;

            double w = pose_data.rotation.w;
            double x = -1.0 * pose_data.rotation.z;
            double y = pose_data.rotation.x;
            double z = -1.0 * pose_data.rotation.y;

            //double pitch = -asin(2.0 * (x * z - w * y)) * (180.0 / M_PI);
            //double roll = atan((2.0 * (w*x + y*z)) /  (w*w - x*x - y*y + z*z)) * (180.0 / M_PI);
            yaw = atan((2.0 * (w*z + x*y)) /  (w*w + x*x - y*y - z*z)) * (180.0 / M_PI);
        }
    };

    // Start the pipeline streaming according to the configuration.
    // The pipeline captures the samples from the device and delivers them
    // to the frame callback. 
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);
    
    double distance_to_destination = 8.0;
    double degrees_to_rotate = 0.0;
    double degrees_actually_rotated = 0.0;

    /*
    // go to coordinates test
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "going to destination" << std::endl;
    go_to_destination(122.0, 23.5, 62.0, 92.0, yaw_to_degrees(yaw, current_angle_quaternion));
    std::cout << "destination reached" << std::endl;
    */

    int iterations = 0;
    while(1) {
      RobotState new_state;
      // check if robot has reached its destination
      // check if task completed
      if(robot.getTask().getStatus()) {
	// status completed
	// load next task
	
      }
      else {
	robot.run();
      }
      
      if(std::fabs(robot.getX() - w1.getX()) < 1.5){
	new_state = STOP;
	
	//robot.setState(STOP);
      }
      else {
	new_state = MOVE_FORWARD;
	//robot.setState(MOVE_FORWARD);
      }


      // execute actions if new state is requested
      // or exeuct
      if(robot.getState() != new_state) {
	robot.setState(new_state);
	robot.run();
      }

      // if robot has reached its destination, check what task it has to perform
      // check if robot has reached pose correction waypoint
      // set the robot's next destination if it has completed its task
      // set the next action the robot is supposed to perform

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
