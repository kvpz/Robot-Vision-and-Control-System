#ifndef MAP_HPP
#define MAP_HPP
#include <optional>
#include "xypoint.hpp"

class Map
{
public:
    Map();

    // getters
    double RobotX() const;

    double RobotY() const;

    XYPoint getRobotCurrentLocation();

    double getRobotOrientation() const;

    double getDestinationOrientation();

    XYPoint getNextDestinationXY();

    // setters
    void setRobotCurrentCoordinate(double x, double y);

    void setRobotOrientation(double o);

    void setDestinationXY(double destx, double desty);

    void setDestinationDesiredOrientation(double theta);

    inline bool getIsEndpointOrientationRequired() { return isEndpointOrientationRequired; };

private:
    // robot current position data
    XYPoint robotCurrentLocation;
    double robotCurrentOrientation;

    // destination position data
    XYPoint destinationXY;
    double destinationOrientation;
    bool isEndpointOrientationRequired;
    // map<size_t, Waypoint> pointsVisited;
};

#endif