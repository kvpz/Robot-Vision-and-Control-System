#ifndef WAYPOINTS_H
#define WAYPOINTS_H


/*
std::pair<float, float> origin(0.0f, 0.0f);
std::pair<float, float> origin_top(0.0f, -0.02f);
std::pair<float, float> bottom_left_attraction(-0.9f, 0.1524f);
std::pair<float, float> top_left_attraction(-0.9f, -0.45f);
std::pair<float, float> top_right_recycling(0.9f, -0.45f);
std::pair<float, float> bottom_right_recycling(0.9f, 0.0f);
*/

// waypoints (inches) based on camera coordinates
/*
std::pair<float, float> origin(0.0f, 0.0f);
std::pair<float, float> origin_top(0.0f, -1.0f);
std::pair<float, float> bottom_left_attraction(-35.0f, 6.0f);//(-41.0f, 7.0f);
std::pair<float, float> top_left_attraction(-35.0f, -22.0f);//(-41.0f, -32.0f);
std::pair<float, float> top_right_recycling(35.0f, -22.0f);//(41.0f, -32.0f);
std::pair<float, float> bottom_right_recycling(35.0f, 1.0f);//(41.0f, 7.0f);
*/

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

  inline double getX() { return _x; }
  inline double getY() { return _y; }

  void setX(double x) { _x = x; }
  void setY(double y) { _y = y; }
private:
  double _x, _y;
};


#endif
