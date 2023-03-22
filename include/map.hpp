#ifndef MAP_HPP
#define MAP_HPP
#include <optional>
#include "waypoint.hpp"

class Map
{
public:
    Map() {}

    // getters
    inline double RobotX() const
    {
        return robotCurrentLocation.getX();
    }

    inline double RobotY() const
    {
        return robotCurrentLocation.getY();
    }

    inline Waypoint getRobotCurrentLocation()
    {
        return robotCurrentLocation;
    }

    inline double getRobotOrientation() const
    {
        return robotCurrentOrientation;
    }

    // Not all destination points require robot to be in a particular orientation.
    // Calling functions can evaluate whether the optional Waypoint object is null
    // by using the std::optional has_value function. 
    inline std::optional<double> getDestinationOrientation()
    {
        return destinationOrientation;
    }

    inline Waypoint getNextDestinationXY()
    {
        return destinationXY;
    }

    // setters
    inline void setRobotCurrentCoordinate(double x, double y)
    {
        robotCurrentLocation.setX(x);
        robotCurrentLocation.setY(y);
    }

    inline void setRobotOrientation(double o)
    {
        robotCurrentOrientation = o;
    }

    inline void setDestinationXY(double destx, double desty) 
    {
        destinationXY.setX(destx);
        destinationXY.setY(desty);
    }

    inline void setDestinationDesiredOrientation(double theta) 
    {
        destinationOrientation = theta;
    }

private:
    // robot current position data
    Waypoint robotCurrentLocation;
    double robotCurrentOrientation;

    // destination position data
    Waypoint destinationXY;
    double destinationOrientation;
    // map<size_t, Waypoint> pointsVisited;
};

#endif