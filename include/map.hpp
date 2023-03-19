#ifndef MAP_HPP
#define MAP_HPP
#include "waypoint.hpp"

class Map
{
public:
    Map(){}
    
    double getRobotCurrentXCoordinatePoint() const
    {
        return robotCurrentLocation.getX();
    }

    double getRobotCurrentYCoordinatePoint() const
    {
        return robotCurrentLocation.getY();
    }

    double getRobotOrientation() const
    {
        return robotCurrentOrientation;
    }

    // setters
    void setRobotCurrentCoordinate(double x, double y) 
    {
      robotCurrentLocation.setX(x);
      robotCurrentLocation.setY(y);
    }

    void setRobotOrientation(double o)
    {
      robotCurrentOrientation = o;
    }

private:
    Waypoint robotCurrentLocation;
    double robotCurrentOrientation; // (gyro) orientation

};

#endif