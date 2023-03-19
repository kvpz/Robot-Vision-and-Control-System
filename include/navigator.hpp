#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "map.hpp"
#include "utility.hpp"

#define NAVDEBUG true

class Navigator
{
public:
    Navigator(){}

    // setters
    double setAngleToDestination(double angle)
    {
        angleToDestination = angle;
    }

    // getters
    double getAngleToDestination() const
    {
        return angleToDestination;
    }

    double getRobotAngleToPoseOrientation(Map* map, double endpointOrientation) const 
    {
        return angleToEndpointOrientation(map->getRobotOrientation(), endpointOrientation);
    }

    double getRobotAngleToPoint(Map* map, double x, double y) const 
    {
          return angleToPoint(map->getRobotCurrentXCoordinatePoint(), map->getRobotCurrentYCoordinatePoint(), x, y, map->getRobotOrientation());
    }

    RobotPoseToWaypoint isRobotOnPath(double robotX, double robotY, double destX, double destY) 
    {
        RobotPoseToWaypoint result = ON_PATH;
        // check if robot position (x,y) approximately near destination
        bool isYapproxnear = approximately(robotY, destY, 2.0);
        bool isXapproxnear = approximately(robotX, destX, 2.0);
        double angleToDestTolerance = 10.0;

        setAngleToDestination((destX, destY));

        if(isYapproxnear && isXapproxnear) {
            // near the waypoint
            result = NEAR;
        }
        else if (getAngleToDestination() < angleToDestTolerance && getAngleToDestination() > -1.0*angleToDestTolerance
                && (!(robotX < (destX - 2.5)) || !(robotX > (destX + 2.5)))) 
        { // robotY > destY && robotX == destX
            // detect if drifting from path
            result = ON_PATH;
        }
        else {
            result = OFF_PATH;
        }    

        if(NAVDEBUG) {
            std::cout << "\n====== isRobotOnPath ======\n";
            std::cout << "result: " << printRobotPoseToWaypoint(result) << "\n";
            std::cout << "(isRobotOnPath) angle to dest: " << getAngleToDestination() << "\n";
            std::cout << "=============================\n" << std::endl;
        }

        //robotPoseToWaypoint = result;
        return result;
    }

    RobotOrientationAtEndpoint isRobotOriented(Map* map, double endpointOrientation) 
    {
        RobotOrientationAtEndpoint result = ORIENTED;
        double tolerance = 5.0;
        bool isRobotApproximatelyOriented = false;

        // robot orientation minus endpoint orientation
        setAngleToDestination(getRobotAngleToPoseOrientation(map, endpointOrientation));
        isRobotApproximatelyOriented = std::fabs(getAngleToDestination()) > tolerance ? false : true;

        if(getAngleToDestination() < 0.0 && isRobotApproximatelyOriented) {
          // rotate CCW if absolute value of difference is less than 180
          if(std::fabs(getAngleToDestination()) < 180.0 && std::fabs(getAngleToDestination()) > tolerance)
            result = OFF_TO_RIGHT;
          else
            result = OFF_TO_LEFT;
        }
        else if(getAngleToDestination() > 0.0 && std::fabs(getAngleToDestination()) > tolerance) {
          // rotate CW if
          if(std::fabs(getAngleToDestination()) < 180.0)
            result = OFF_TO_LEFT;
          else
            result = OFF_TO_RIGHT;
        }

        return result;  
    }

private:
    double angleToDestination;
    Waypoint destination;

};

#endif