#ifndef WAYPOINT_H
#define WAYPOINT_H

/*
  This class resembles a xy-coordinate point a robot must travel to.
*/
class Waypoint
{
public:
  // constructors
  Waypoint() = default;
  Waypoint(double x, double y) : _x(x), _y(y) {}

  // getters
  inline double getX() const { return _x; }
  inline double getY() const { return _y; }

  // setters
  void setX(double x) { _x = x; }
  void setY(double y) { _y = y; }

private:
  // private member data
  double _x, _y;
};

#endif

/*
  NOTES:

  This class's name should probably be changed to XYPoint because that
  is how it's currently being used for. The class instances are used to
  represent (1) the robot's current location in the global map, 
  (2) the xy-coordinate points of the destination, (3) and potentially even 
  in a Map class member data structure for tracking visited xy-coordinate points, 
  (4) etc.


*/