#ifndef MAP_HPP
#define MAP_HPP
#include <optional>
#include <map>
#include <unordered_map>
#include "xypoint.hpp"
#include "enums/objects.hpp"
#include "enums/attractionColors.hpp"

struct XYPointComparator {
    bool operator()(const XYPoint& xy1, const XYPoint& xy2) {
        bool isXDiffNear = approximately(xy1.getX(), xy2.getX(), 3.0);
        bool isYDiffNear = approximately(xy1.getY(), xy2.getY(), 3.0);
        if(isXDiffNear || isYDiffNear) {
            // don't map
            return false;
        }
        else {
            return true;
        }
    }
};

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
    
    AttractionColors getBottomLeftAttractionColor() {
        return bottomLeftAttractionColor;
    }

    AttractionColors getTopLeftAttractionColor() {
        return topLeftAttractionColor;
    }

    bool isAttractionColorKnown() const
    {
        return bottomLeftAttractionColor != AttractionColors::NONE || 
            topLeftAttractionColor != AttractionColors::NONE;
    }

    // setters
    void setRobotCurrentCoordinate(double x, double y);

    void setRobotOrientation(double o);

    void setDestinationXY(double destx, double desty);

    void setDestinationDesiredOrientation(double theta);

    inline bool getIsEndpointOrientationRequired() { return isEndpointOrientationRequired; };

    void setIsEndpointOrientationRequired(bool value) ;

    void setBottomLeftAttractionColor(AttractionColors attractionColor) {
        bottomLeftAttractionColor = attractionColor;
    }

    void setTopLeftAttractionColor(AttractionColors attractionColor) {
        topLeftAttractionColor = attractionColor;
    }

    bool addObjectDetected(ObjectType, XYPoint);   
    //void deleteObject(XYPoint);

    std::multimap<XYPoint, ObjectType, XYPointComparator> getOccupancyGrid() { return occupancyGrid; }
    //std::unordered_multimap<ObjectType, XYPoint> getObjectMap() { return objectMap; }

private:
    // robot current position data
    XYPoint robotCurrentLocation;
    double robotCurrentOrientation;

    // destination position data
    XYPoint destinationXY;
    double destinationOrientation;
    bool isEndpointOrientationRequired;
    
    std::map<size_t, XYPoint> pointsVisited;

    std::multimap<XYPoint, ObjectType, XYPointComparator> occupancyGrid;
    //std::unordered_multimap<ObjectType, XYPoint> objectMap;

    // data about areas of interest
    AttractionColors bottomLeftAttractionColor;
    AttractionColors topLeftAttractionColor;
    
};

#endif