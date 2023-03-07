#ifndef WAYPOINTS_H
#define WAYPOINTS_H


// waypoints in centimeters (x,y)
std::pair<double, double> origin(0.0f, 0.0f);
std::pair<double, double> origin_top(5.08f, 0.0f);
std::pair<double, double> bottom_left_attraction(-35.0f, 6.0f);//(-41.0f, 7.0f);
std::pair<double, double> top_left_attraction(-35.0f, -22.0f);//(-41.0f, -32.0f);
std::pair<double, double> top_right_recycling(35.0f, -22.0f);//(41.0f, -32.0f);
std::pair<double, double> bottom_right_recycling(35.0f, 1.0f);//(41.0f, 7.0f);

std::vector<std::pair<double,double>> waypoints({
    origin_top,
    bottom_left_attraction, 
    top_left_attraction, 
    top_right_recycling, 
    bottom_right_recycling
});



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
