#ifndef UTILITY_HPP
#define UTILTY_HPP
#include "includes.hpp"
#include "robot_specs.hpp"
#include "task.hpp"

#define UTILITYDEBUG false

// *********************************
// Math related functions
// *********************************
static inline bool approximately(double current, double expected, double precision)
{
  return (current > (expected - precision)) && (current < (expected + precision));
}

static int quadrant_identifier(double angle)
{
  int result;
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

/*
  Calculate the angle from one point to another on a cartesian plane. 
  This is done by using the point slope formula then taking the 
  inverse tangent of the slope. The robot's orientation in the global map is known
  so the robot's orientation is subtracted from the slope angle to obtain 'theta'. 
  If 'theta' < 0 and abs('theta') is greater than 180, then 360 is added to theta; 
  otherwise if 'theta' > 180 then 360 is subtracted from 'theta'. 

  'theta' is returned.
 */
static double angleToPoint(double x_robot, double y_robot, double x_destination, double y_destination, double robot_current_angle)
{
  double x_diff = x_destination - x_robot;
  double y_diff = y_destination - y_robot;
  double beta = 0.0;
  double theta = 0.0;

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
    beta = beta + 360.0;
  }

  theta = beta - robot_current_angle;

  if(theta < 0.0) {
    if(std::fabs(theta) > 180.0) {
      theta = 360 + theta;
    }
  }
  else if(theta > 180.0) {
    //if(theta > 180.0) {
      theta = theta - 360.0;
      //}
  }

  if(UTILITYDEBUG) {
    std::cout << "\n=============================================\n";
    std::cout << "(angleToPoint) x_destination: " << x_destination << "\n";
    std::cout << "(angleToPoint) y_destination: " << y_destination << "\n";
    std::cout << "(angleToPoint) x_robot: " << x_robot << "\n";
    std::cout << "(angleToPoint) y_robot: " << y_robot << "\n";
    std::cout << "(angleToPoint) y_diff: " << y_diff << "\n";
    std::cout << "(angleToPoint) x_diff: " << x_diff << "\n";
    std::cout << "(angleToPoint) y_diff / x_diff: " << y_diff / x_diff << "\n";
    std::cout << "(angleToPoint) beta: " << beta << "\n";
    std::cout << "(angleToPoint) current_angle: " << robot_current_angle << "\n";
    std::cout << "=============================================" << std::endl;
  }

  return theta;
}

/*
  This function calculates the angle between the robots current orientation and the
  orientation required at the endpoint. It is assumed that the robot is at the endpoint
  when this function is called hence the name. 

  The angle is calculated by 
 */
static double angleToEndpointOrientation(double robotOrientation, double endpointOrientation)
{
  // this information alone will not tell us if robot should rotate left or right
  return robotOrientation - endpointOrientation;
}

#endif
