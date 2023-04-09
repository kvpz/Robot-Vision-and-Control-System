#include "map.hpp"

Map::Map()
    : bottomLeftAttractionColor(AttractionColors::NONE), topLeftAttractionColor(AttractionColors::NONE)
{}

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
    //isEndpointOrientationRequired = false;
}

void Map::setDestinationXY(XYPoint xypoint) 
{
    destinationXY = xypoint;
    //isEndpointOrientationRequired = false;
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

bool Map::addObjectDetected(ObjectType objectType, const XYPoint& xypoint)
{
    bool isObjectInMap = false;

    for(const auto& a : occupancyGrid) {
        XYPoint xy = a.first;

        //bool isXDiffNear = approximately(xypoint.getX(), xy.getX(), 3.0);
        //bool isYDiffNear = approximately(xypoint.getY(), xy.getY(), 3.0);
        double precision = 3.0;
        bool isXDiffNear = (xypoint.getX() > (xy.getX() - precision)) && (xypoint.getX() < (xy.getX() + precision));
        bool isYDiffNear = (xypoint.getY() > (xy.getY() - precision)) && (xypoint.getY() < (xy.getY() + precision));
        
        if(isXDiffNear && isYDiffNear) {
            isObjectInMap = true;
            return false;
            break;
        }
    }

    if(!isObjectInMap) {
        auto itr = occupancyGrid.insert(std::pair(xypoint, objectType));
        objectMap.emplace(std::pair(objectType, xypoint));
        if(itr != occupancyGrid.end())
            return true;
        else    
            return false;
    }
    else {
        return false;
    }


    
}
