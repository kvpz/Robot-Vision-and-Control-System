#ifndef MAP_HPP
#define MAP_HPP
#include <optional>
#include <map>
#include <unordered_map>
#include "xypoint.hpp"
#include "objects.hpp"

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

    void setIsEndpointOrientationRequired(bool value) ;

private:
    // robot current position data
    XYPoint robotCurrentLocation;
    double robotCurrentOrientation;

    // destination position data
    XYPoint destinationXY;
    double destinationOrientation;
    bool isEndpointOrientationRequired;
    
    std::map<size_t, XYPoint> pointsVisited;
    //std::unordered_map<std::pair<int, int>, ObjectType> occupancyGrid;

};

#endif