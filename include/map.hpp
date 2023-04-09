#ifndef MAP_HPP
#define MAP_HPP
#include <iostream>
#include <optional>
#include <map>
#include <unordered_map>
#include <functional>
#include "xypoint.hpp"
#include "enums/objects.hpp"
#include "enums/attractionColors.hpp"
#include "utility.hpp"

struct XYPointComparator {
    bool operator()(const XYPoint& xy1, const XYPoint& xy2) const {
        //bool isXDiffNear = approximately(xy1.getX(), xy2.getX(), 3.0);
        //bool isYDiffNear = approximately(xy1.getY(), xy2.getY(), 3.0);
        /*
        double precision = 3.0;
        bool isXDiffNear = (xy1.getX() > (xy2.getX() - precision)) && (xy1.getX() < (xy2.getX() + precision));
        bool isYDiffNear = (xy1.getY() > (xy2.getY() - precision)) && (xy1.getY() < (xy2.getY() + precision));
        if(isXDiffNear && isYDiffNear) {
            std::cout << "(xypointcomparator) " << xy1 << " exists in map" << std::endl;
            // don't map
            return false;
        }
        else {
            std::cout << "(xypointcomparator) " << xy2 << " DOES NOT exist in map" << std::endl;
            return true;
        }
        */

        if(xy1.getX() < xy2.getX())
            return true;
        else
            return false;

    }
};

struct ObjectTypeHash {
    std::size_t operator()(const ObjectType& obj) const {
        return std::hash<int>()(static_cast<int>(obj.getObjectType()));
    }
};

struct ObjectTypeEqual {
    bool operator()(const ObjectType& lhs, const ObjectType& rhs) const {
        return lhs.getObjectType() == rhs.getObjectType();
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
    void setDestinationXY(XYPoint xypoint);

    void setDestinationDesiredOrientation(double theta);

    inline bool getIsEndpointOrientationRequired() { return isEndpointOrientationRequired; };

    void setIsEndpointOrientationRequired(bool value) ;

    void setBottomLeftAttractionColor(AttractionColors attractionColor) {
        bottomLeftAttractionColor = attractionColor;
    }

    void setTopLeftAttractionColor(AttractionColors attractionColor) {
        topLeftAttractionColor = attractionColor;
    }

    bool addObjectDetected(ObjectType, const XYPoint&);   
    //void deleteObject(XYPoint);

    std::multimap<XYPoint, ObjectType, XYPointComparator> getOccupancyGrid() { return occupancyGrid; }
    //std::unordered_multimap<ObjectType, XYPoint> getObjectMap() { return objectMap; }

    void printOccupancyGrid() 
    {
        std::cout << "====== occupancy grid ========" << std::endl;
        for(auto& og : occupancyGrid) {
            std::cout << og.first << " --- " << og.second << std::endl;
        }
    }

    void printObjectMap() 
    {
        std::cout << "====== object map ========" << std::endl;
        for(auto& og : objectMap) {
            std::cout << og.first << " --- " << og.second << std::endl;
        }
    }

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
    std::unordered_multimap<ObjectType, XYPoint, ObjectTypeHash, ObjectTypeEqual> objectMap;

    // data about areas of interest
    AttractionColors bottomLeftAttractionColor;
    AttractionColors topLeftAttractionColor;
    
};

#endif