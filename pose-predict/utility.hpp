#ifndef UTILITY_HPP
#define UTILTY_HPP
#include "includes.hpp"
#include "robot_specs.hpp"

#define SLEEP_TIME 250
#define INCHES_IN_METER 39.3701

static void move_forward(double distance);
static void stop();
static void send_command(std::string message);
static inline bool approximately(float current, float expected, float precision, bool isGyro);
static double convert_to_degrees(double val);

// *********************************
// Robot related functions
// *********************************
static void move_forward(double distance, double robot_speed_per_sec)
{
  std::cout << "move forward" << std::endl;
  send_command("F");
  //std::cout << "Sleep duration: " << (long)(distance / robot_speed_per_sec * 1000.0) << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds((long)(distance / robot_speed_per_sec * 1000.0)));
  stop();
}

static void move_backward(double distance, double robot_speed_per_sec)
{
  std::cout << "move backward" << std::endl;
  send_command("B");
  //std::cout << "Sleep duration: " << (long)(distance / robot_speed_per_sec * 1000.0) << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds((long)(distance / robot_speed_per_sec * 1000.0)));
  stop();
}

static void rotate_CW(double degrees, double robot_speed)
{
  std::cout << "rotate_CW ( " << degrees << " )" << std::endl;
  send_command("C");
  //std::cout << "Sleep duration: " << (long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees) << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds((long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees)));
  stop();
}

static void rotate_CCW(double degrees, double robot_speed)
{
  std::cout << "rotate_CCW ( " << degrees << " )" << std::endl;
  send_command("Z");
  //std::cout << "Sleep duration: " << (long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees) << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds((long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees)));
  stop();
}

static void stop()
{
  send_command("S");
}

// *********************************
// Communications related functions
// *********************************
static void send_command(std::string message) 
{
  std::cout << "(RELEASE) Serial message: " << message << std::endl;
  boost::asio::io_service io;
  boost::asio::serial_port serial(io, "/dev/ttyACM0");

  // Set the baud rate, character size, flow control, and parity options.
  serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
  serial.set_option(boost::asio::serial_port_base::character_size(8));
  serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

  boost::asio::write(serial, boost::asio::buffer(message.c_str(), message.size()));
}

// *********************************
// Math related functions
// *********************************
static inline bool approximately(double current, double expected, double precision, bool isGyro)
{

  if(isGyro) {
    std::cout << "Gyro current:  " << current << std::endl;
    std::cout << "Gyro expected: " << expected << std::endl;
  }
  else {
    std::cout << "accelerometer current:  " << current << std::endl;
    std::cout << "accelerometer expected: " << expected << std::endl;
  }
    
  return (current > (expected - precision)) && (current < (expected + precision));
}

static double convert_quaternions_to_degrees(double val)
{
  //std::cout << "convert_to_degrees : " << 270 - 2.0 * std::abs(std::acos(val)) * 180.0 / M_PI  << std::endl;
  double alpha = 270 - 2.0 * std::acos(val) * 180 / M_PI;
  if (alpha < 0 ){
    alpha = 360 +  alpha;
    return alpha;
  }

  return alpha;
}

static int quadrant_identifier(double angle)
{
  if(angle > 0.0 && angle < 90.0)         return 1;
  else if(angle > 90.0 && angle < 180.0)  return 2;
  else if(angle > 180.0 && angle < 270.0) return 3;
  else                                    return 4;
}

static int quadrant_identifier(double yaw, double quat_y)
{
  if(yaw > 0.0 && std::fabs(quat_y) < 0.707)         return 1;
  else if(yaw < 0.0 && std::fabs(quat_y) < 0.707)    return 2;
  else if(yaw > 0.0 && std::fabs(quat_y))            return 3;
  else                                               return 4;
}

static double yaw_to_degrees(double yaw, double quat_y)
{
  switch(quadrant_identifier(yaw, quat_y)) {
  case 1:
    return 90.0 - yaw;
  case 2:
    return 90.0 - yaw;
  case 3:
    return 270.0 - yaw;
  case 4:
    return 270.0 - yaw;
  }

  return 0.0;
}

static void go_to_destination(double x_robot, double y_robot, double x_destination, double y_destination, double robot_current_angle)
{
  double x_diff = x_destination - x_robot;
  double y_diff = y_destination - y_robot;
  double beta = 0.0;
  double theta = 0.0;
  double distance = 0.0;

  beta = atan(y_diff / x_diff) * 180.0 / (M_PI);
  
  if(x_diff > 0.0 && y_diff > 0.0){
    // first quadrant

  }
  else if(x_diff < 0.0 && y_diff > 0.0) {
    // second quadrant
    beta = beta + 180.0;
  }
  else if(x_diff < 0.0 && y_diff < 0.0) {
    // third quadrant
    beta = beta + 180.0;
  }
  else {
    // fourth quadrant
    beta = beta + 270.0;
  }

  theta = beta - robot_current_angle;
  distance = sqrt(x_diff * x_diff + y_diff * y_diff);
  std::cout << "robot current angle:  " << robot_current_angle << std::endl;
  std::cout << "distance: " << distance << std::endl;
  std::cout << "x2 - x1: " << x_diff << std::endl;
  std::cout << "y2 - y1: " << y_diff << std::endl;
  std::cout << "beta: " << beta << std::endl;
  std::cout << "theta: " << theta << std::endl;
  
  if(theta < 0.0) {
    // go clockwise
    rotate_CW(theta, ROBOT_360_ROTATE_TIME_MILLISEC);
  }
  else {
    // go counterclockwise
    rotate_CCW(theta, ROBOT_360_ROTATE_TIME_MILLISEC);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  move_forward(distance, ROBOT_SPEED_CM_PER_SEC);

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}


#endif
