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

template<typename T = double>
struct XYPoint
{
  T x = 0.0;
  T y = 0.0;

  inline T getX() const { return x; }
  inline T getY() const { return y; }

  inline void setX(T _x) { x = _x; }
  inline void setY(T _y) { y = _y; }

  // constructors
  XYPoint() = default;
  XYPoint(T x, T y) : x(x), y(y) {}

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

template<typename T>
inline std::ostream& operator<<(std::ostream& os, const XYPoint<T>& obj) {
  os << "(" << obj.x << ", " << obj.y << ")";
  return os;
}

#endif


/*
  NOTES:

  This class's name should probably be changed to XYPoint because that
  is how it's currently being used for. 


*/