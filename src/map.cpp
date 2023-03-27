#include "map.hpp"

Map::Map() {}

// getters
double Map::RobotX() const
{
    return robotCurrentLocation.getX();
}

double Map::RobotY() const
{
    return robotCurrentLocation.getY();
}

XYPoint Map::getRobotCurrentLocation()
{
    return robotCurrentLocation;
}

double Map::getRobotOrientation() const
{
    return robotCurrentOrientation;
}

// Not all destination points require robot to be in a particular orientation.
// Calling functions can evaluate whether the optional Waypoint object is null
// by using the std::optional has_value function. 
double Map::getDestinationOrientation()
{
    return destinationOrientation;
}

XYPoint Map::getNextDestinationXY()
{
    return destinationXY;
}

// setters
void Map::setRobotCurrentCoordinate(double x, double y)
{
    robotCurrentLocation.setX(x);
    robotCurrentLocation.setY(y);
}

void Map::setRobotOrientation(double o)
{
    robotCurrentOrientation = o;
}

void Map::setDestinationXY(double destx, double desty) 
{
    destinationXY.setX(destx);
    destinationXY.setY(desty);
    isEndpointOrientationRequired = false;
}

void Map::setDestinationDesiredOrientation(double theta) 
{
    destinationOrientation = theta;
    isEndpointOrientationRequired = true;
}

void Map::setIsEndpointOrientationRequired(bool value) 
{
    isEndpointOrientationRequired = value;
}