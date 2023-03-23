#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP
#include <iostream>
#include <memory>
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "map.hpp"
#include "utility.hpp"

#define NAVDEBUG true

class Navigator
{
public:
    Navigator();

    // setters
    void setAngleToDestination(double angle);

    // getters
    double getAngleToDestination() const;

    double getRobotToEndpointSlopeAngle(std::shared_ptr<Map> map, double endpointDesiredOrientation) const;

    /*
        Calculate the angle from one point to another on a cartesian plane. 
        This is done by using the point slope formula then taking the 
        inverse tangent of the slope. The robot's orientation in the global map is known
        so the robot's orientation is subtracted from the slope angle to obtain 'theta'. 
        If 'theta' < 0 and abs('theta') is greater than 180, then 360 is added to theta; 
        otherwise if 'theta' > 180 then 360 is subtracted from 'theta'. 

        'theta' is returned. 'theta' is used in practice to represent the difference
        between the robot's current orientation and the desired orientation. 
        TODO: rename function to something more meaningful like robotAngularDistanceToOrientation
    */
    double robotAngularDistanceToOrientation(std::shared_ptr<Map> map);

    RobotPoseToWaypoint isRobotOnPath(std::shared_ptr<Map> map, double robotX, double robotY, double destX, double destY);

    RobotOrientationAtEndpoint isRobotOriented(std::shared_ptr<Map> map, double endpointOrientation);

private:
    double angleToDestination;
};

#endif