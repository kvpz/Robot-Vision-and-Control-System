/*
  This class resembles a xy-coordinate point a robot must travel to.
  The class instances are used to:
  (1) represent the robot's current location in the global map, 
  (2) the xy-coordinate points of the destination, 
  (3) tracking visited coordinate points in a Map, 
  (4) etc.
*/
#ifndef XYPOINT_H
#define XYPOINT_H
#include <iostream>

struct XYPoint
{
  double x = 0.0;
  double y = 0.0;

  inline double getX() const { return x; }
  inline double getY() const { return y; }

  inline void setX(double _x) { x = _x; }
  inline void setY(double _y) { y = _y; }

  // constructors
  XYPoint() = default;
  XYPoint(double x, double y) : x(x), y(y) {}

  // copy constructor
  XYPoint(const XYPoint& other) : x(other.x), y(other.y) {}

  // assignment operator
  XYPoint& operator=(const XYPoint& other) {
    if (this != &other) {
      x = other.x;
      y = other.y;
    }
    return *this;
  }
};

inline std::ostream& operator<<(std::ostream& os, const XYPoint& obj) {
  os << "(" << obj.x << ", " << obj.y << ")";
  return os;
}

#endif


/*
  NOTES:

  This class's name should probably be changed to XYPoint because that
  is how it's currently being used for. 


*/