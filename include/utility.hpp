#ifndef UTILITY_HPP
#define UTILTY_HPP
#include <cmath>

// *********************************
// Math related functions
// *********************************
static inline bool approximately(double current, double expected, double precision)
{
  return (current > (expected - precision)) && (current < (expected + precision));
}

static inline double distance(double x1, double x2, double y1, double y2)
{
  return sqrt(std::pow(x2 - x1, 2.0) + std::pow(y2 - y1, 2.0));
}

static inline double distance(XYPoint robotxy, XYPoint destinationxy) 
{
  return sqrt(std::pow(destinationxy.getX() - robotxy.getX(), 2.0) + std::pow(destinationxy.getY() - robotxy.getY(), 2.0));
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
  This function calculates the angle between the robots current orientation and the
  orientation required at the endpoint. It is assumed that the robot is at the endpoint
  when this function is called hence the name. 

  The angle is calculated by 
 */
static double angleToEndpointOrientation(double robotOrientation, double endpointDesiredOrientation)
{
  // this information alone will not tell us if robot should rotate left or right
  return robotOrientation - endpointDesiredOrientation;
}

#endif
