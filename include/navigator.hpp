#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP
#include <iostream>
#include <memory>
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "map.hpp"
#include "utility.hpp"

#define NAVDEBUG false

class Navigator
{
public:
    Navigator();

    // setters
    void setAngleToDestination(double angle);

    // getters
    double getAngleToDestination() const;

    double getRobotToEndpointSlopeAngle(std::shared_ptr<Map> map, double endpointDesiredOrientation) const;

    double robotAngularDistanceToOrientation(std::shared_ptr<Map> map);

    RobotPoseToWaypoint isRobotOnPath(std::shared_ptr<Map> map, double robotX, double robotY, double destX, double destY);

    RobotOrientationAtEndpoint isRobotOriented(std::shared_ptr<Map> map, double endpointOrientation);

private:
    double angleToDestination;
};

#endif