#ifndef UTILITY_HPP
#define UTILTY_HPP
#include "includes.hpp"
#include "robot_specs.hpp"

#define SLEEP_TIME 250
#define INCHES_IN_METER 39.3701
#define UTILITYDEBUG false


// *********************************
// Math related functions
// *********************************
static inline bool approximately(double current, double expected, double precision, bool isGyro)
{
  if(UTILITYDEBUG){
    if(isGyro) {
      std::cout << "(approximately) Gyro current:  " << current << std::endl;
      std::cout << "(approximately) Gyro expected: " << expected << std::endl;
    }
    else {
      std::cout << "(approximately) accelerometer current:  " << current << std::endl;
      std::cout << "(approximately) accelerometer expected: " << expected << std::endl;
    }
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
  double result;
  if(angle > 0.0 && angle < 90.0)         result = 1;
  else if(angle > 90.0 && angle < 180.0)  result = 2;
  else if(angle > 180.0 && angle < 270.0) result = 3;
  else                                    result = 4;

  //std::cout << "(quadrant_identifier(yaw)) xy-quadrant: " << result << "\n" << std::endl;
  return result;
}

static int quadrant_identifier(double yaw, double quat_y)
{
  int result;
  if(yaw > 0.0 && std::fabs(quat_y) < 0.707)         result = 1;
  else if(yaw < 0.0 && std::fabs(quat_y) < 0.707)    result = 2;
  else if(yaw > 0.0 && std::fabs(quat_y))            result = 3;
  else                                               result = 4;

  //std::cout << "(quadrant_identifier(yaw, quat)) xy-quadrant: " << result << "\n" << std::endl;
  return result;
}

/* 
  This one returns values between -90 and 90.
*/
static double yaw_to_degrees(double yaw, double quat_y)
{
  int quadrant = quadrant_identifier(yaw, quat_y);

  switch(quadrant) {
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

static double angleToPoint(double x_robot, double y_robot, double x_destination, double y_destination, double robot_current_angle)
{
  double x_diff = x_destination - x_robot;
  double y_diff = y_destination - y_robot;
  double beta = 0.0;
  double theta = 0.0;
  double distance = 0.0;

  if(UTILITYDEBUG) {
    std::cout << "(angleToPoint) x_destination: " << x_destination << "\n";
    std::cout << "(angleToPoint) y_destination: " << y_destination << "\n";
    std::cout << "(angleToPoint) x_robot: " << x_robot << "\n";
    std::cout << "(angleToPoint) y_robot: " << y_robot << "\n";
    std::cout << "(angleToPoint) y_diff: " << y_diff << "\n";
    std::cout << "(angleToPoint) x_diff: " << x_diff << "\n";
    std::cout << "(angleToPoint) y_diff / x_diff: " << y_diff / x_diff << "\n";
    std::cout << "(angleToPoint) beta: " << beta << "\n";
    std::cout << "(angleToPoint) current_angle: " << robot_current_angle << std::endl;
  }

  beta = atan(y_diff / x_diff) * 180.0 / (M_PI); // erroneos
  
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

  return theta;
}

#endif
