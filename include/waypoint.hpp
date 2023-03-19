#ifndef WAYPOINT_H
#define WAYPOINT_H


class Waypoint
{
public:
  Waypoint() = default;
  Waypoint(double x, double y) : _x(x), _y(y){}

  inline double getX() const { return _x; }
  inline double getY() const { return _y; }

  void setX(double x) { _x = x; }
  void setY(double y) { _y = y; }
private:
  double _x, _y;
};


#endif
