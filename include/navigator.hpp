#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP
<<<<<<< HEAD
=======
#include <iostream>
#include <memory>
#include "enums/travelDirection.hpp"
//#include "robot.hpp"
>>>>>>> kevin_dev_pickupobject_task
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "map.hpp"
#include "utility.hpp"
#include "settings.hpp"

class Navigator
{
public:
    Navigator();

    double getRobotToEndpointSlopeAngle(std::shared_ptr<Map> map, double endpointDesiredOrientation) const;

    double robotAngularDistanceToEndpoint(std::shared_ptr<Map> map, bool reverse);

    RobotPoseToWaypoint isRobotOnPath(const std::shared_ptr<Map> map);

    bool isRobotOriented(std::shared_ptr<Map> map);

    RobotOrientationAtEndpoint getRobotOrientationAtEndpoint(std::shared_ptr<Map> map);

    double getAngleToDestination(std::shared_ptr<Map> map);

    void setTravelDirection(TravelDirection direction) { travelDirection = direction; }
    
    TravelDirection getTravelDirection() { return travelDirection; }

    bool isRobotNearPoint(std::shared_ptr<Map> map);

private:
    TravelDirection travelDirection;

};

#endif
