#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP
#include <iostream>
#include <memory>
#include "enums/travelDirection.hpp"
//#include "robot.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "map.hpp"
#include "utility.hpp"

#define NAVDEBUG true

#define ORIENTATION_RANGE_TOLERANCE 2.0

class Navigator
{
public:
    Navigator();

    double getRobotToEndpointSlopeAngle(std::shared_ptr<Map> map, double endpointDesiredOrientation) const;

    double robotAngularDistanceToEndpoint(std::shared_ptr<Map> map, bool reverse);

    RobotPoseToWaypoint isRobotOnPath(const std::shared_ptr<Map> map);

    //RobotOrientationAtEndpoint 
    bool isRobotOriented(std::shared_ptr<Map> map); //, double endpointOrientation);

    RobotOrientationAtEndpoint getRobotOrientationToEndpoint(std::shared_ptr<Map> map);

    double getAngleToDestination(std::shared_ptr<Map> map);

    void setTravelDirection(TravelDirection direction) { travelDirection = direction; }
    
    TravelDirection getTravelDirection() { return travelDirection; }

private:
    TravelDirection travelDirection;
};

#endif