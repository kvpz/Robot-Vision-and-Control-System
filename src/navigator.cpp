#include "navigator.hpp"

Navigator::Navigator(){}

//double getRobotAngleToPoseOrientation(std::shared_ptr<Map> map, double endpointOrientation) const 
double Navigator::getRobotToEndpointSlopeAngle(std::shared_ptr<Map> map, 
                                               double endpointDesiredOrientation) const 
{
    return map->getRobotOrientation() - map->getDestinationOrientation();//endpointDesiredOrientation;
    //return angleToEndpointOrientation(map->getRobotOrientation(), endpointOrientation);
}

double Navigator::getAngleToDestination(std::shared_ptr<Map> map) 
{
    return std::fmod(
        std::fmod((map->getRobotOrientation() - map->getDestinationOrientation()), 360.0) + 540.0, 360.0) - 180.0;    
}

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
double Navigator::robotAngularDistanceToEndpoint(std::shared_ptr<Map> map, bool reverse = false)
{
    double delta_x = map->getNextDestinationXY().getX() - map->RobotX();
    double delta_y = map->getNextDestinationXY().getY() - map->RobotY();

    double beta = 0.0;
    double theta = 0.0;

    // Calculate the angle of the line connecting the 
    // robot's xy-position to the xy-destination. 
    // Beta will be equal to a value in the [-90, 90] degree range
    beta = atan(delta_y / delta_x) * 180.0 / (M_PI); 
    
    // Adjust beta depending on the quadrant of the destination point relative to the robot's position.
    // Beta will then have a degree value (0,360) taking into account whether the        
    if(delta_x > 0.0 && delta_y > 0.0){
        // first quadrant
        // no adjustments to 
    }
    else if(delta_x < 0.0 && delta_y > 0.0) { // beta < 0 || beta
        // second quadrant
        beta = beta + 180.0;
    }
    else if(delta_x < 0.0 && delta_y < 0.0) {
        // third quadrant
        beta = beta + 180.0;
    }
    else { // (delta_x > 0.0 && delta_y < 0.0) 
        // fourth quadrant
        beta = beta + 360.0;
    }

    
    if (travelDirection == TravelDirection::backward){
        if (map->getRobotOrientation() < 180.0){
            theta = beta - (map->getRobotOrientation() + 180.0);
        }
        else{
            theta = beta - (map->getRobotOrientation() - 180.0);
        }
    }
    else if(travelDirection == TravelDirection::forward){
        theta = beta - map->getRobotOrientation();
    }
    

    //theta = beta - map->getRobotOrientation();
    /*
    if(theta < 0.0) {
        if(std::fabs(theta) > 180.0) {
            theta = 360 + theta;
        }
    }
    else if(theta > 180.0) {
        //if(theta > 180.0) { // robot works better when this line is commented
        theta = theta - 360.0;
        //}
    }
    */

    if(NAVDEBUG) {
        std::cout << "\n============== Navigator::angleToPoint ===================\n";
        std::cout << "x_destination: " << map->getNextDestinationXY().getX() << "\n";
        std::cout << "y_destination: " << map->getNextDestinationXY().getY() << "\n";
        std::cout << "x_robot: " << map->getRobotCurrentLocation().getX() << "\n";
        std::cout << "y_robot: " << map->getRobotCurrentLocation().getY() << "\n";
        std::cout << "delta_y: " << delta_y << "\n";
        std::cout << "delta_x: " << delta_x << "\n";
        std::cout << "delta_y / delta_x: " << delta_y / delta_x << "\n";
        std::cout << "beta: " << beta << "\n";
        std::cout << "theta: " << theta << "\n";
        std::cout << "============================================================" << std::endl;
    }

    return theta;
}

RobotPoseToWaypoint Navigator::isRobotOnPath(const std::shared_ptr<Map> map) 
{
    // check if robot position (x,y) approximately near destination
    bool isYapproxnear = approximately(map->getRobotCurrentLocation().getY(), map->getNextDestinationXY().getY(), ORIENTATION_RANGE_TOLERANCE);
    bool isXapproxnear = approximately(map->getRobotCurrentLocation().getX(), map->getNextDestinationXY().getX(), ORIENTATION_RANGE_TOLERANCE);
    RobotPoseToWaypoint result = ON_PATH;

    double angleToDestination = robotAngularDistanceToEndpoint(map, travelDirection == TravelDirection::backward);// getRobotAngleToPoint(map, destX, destY);

    if(isYapproxnear && isXapproxnear) {
        // near the waypoint
        result = NEAR;
    }
    else if (angleToDestination < ORIENTATION_RANGE_TOLERANCE 
            && angleToDestination > -1.0*ORIENTATION_RANGE_TOLERANCE)
            //&& (!(map->RobotX() < (map->getNextDestinationXY().getX() - 2.5)) 
            //|| !(map->RobotX() > (map->getNextDestinationXY().getX() + 2.5)))) 
    { // robotY > destY && robotX == destX
        // detect if drifting from path
        result = ON_PATH;
    }
    else {
        result = OFF_PATH;
    }    

    if(NAVDEBUG) {
        std::cout << "\n====== Navigator::isRobotOnPath ======\n";
        std::cout << "robot pose relative to waypoint: " << printRobotPoseToWaypoint(result) << "\n";
        std::cout << "(isRobotOnPath) angle to dest: " << angleToDestination << "\n";
        std::cout << "=============================\n" << std::endl;
    }

    return result;
}

/*
    This function returns true if the robot is in the intended
    endpoint orientation. 
*/
bool Navigator::isRobotOriented(std::shared_ptr<Map> map) 
{
    double tolerance = ORIENTATION_RANGE_TOLERANCE;
    bool isRobotApproximatelyOriented = false;

    double angleToDestination = std::fmod(
        std::fmod((map->getRobotOrientation() - map->getDestinationOrientation()), 360.0) + 540.0, 360.0) - 180.0;
    
    isRobotApproximatelyOriented = std::fabs(angleToDestination) > tolerance ? false : true;

    if(NAVDEBUG) {
        std::cout << "======== Navigator::isRobotOriented =========\n";
        std::cout << "angle to destination: " << angleToDestination << "\n";
        std::cout << "approximately oriented? " << isRobotApproximatelyOriented << "\n";
        std::cout << "=============================================\n" << std::endl;
    }

    return isRobotApproximatelyOriented;  
}

RobotOrientationAtEndpoint Navigator::getRobotOrientationToEndpoint(std::shared_ptr<Map> map) 
{
    RobotOrientationAtEndpoint result = ORIENTED;
    double angleToDestination = std::fmod(
        std::fmod((map->getRobotOrientation() - map->getDestinationOrientation()), 360.0) + 540.0, 360.0) - 180.0;
    
    if(approximately(map->getDestinationOrientation(), map->getRobotOrientation(), ORIENTATION_RANGE_TOLERANCE)) {
        result = ORIENTED;
    }
    else if(angleToDestination < 0.0) {
        result = OFF_TO_RIGHT;
    }
    else {
        result = OFF_TO_LEFT;
    }

    if(NAVDEBUG) {
        std::cout << "========= Navigator::getRobotOrientationToEndpoint =========\n";
        std::cout << "result: " << RobotOrientationAtEndpointToString(result) << "\n";
        std::cout << "error tolerance: " << ORIENTATION_RANGE_TOLERANCE << "\n";
        std::cout << "angle to destination: " << angleToDestination << "\n";
        std::cout << "============================================================\n" << std::endl; 
    }

    return result;
}