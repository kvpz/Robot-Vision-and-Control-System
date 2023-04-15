#ifndef BOUNDINGBOX_HPP
#define BOUNDINGBOX_HPP

class BoundingBox {
public:
<<<<<<< HEAD
    BoundingBox(XYPoint<int> _xy1, XYPoint<int> _xy2, ObjectType _objectType)
    : xy1(_xy1), xy2(_xy2), objectType(_objectType)
    {}
    
=======
    BoundingBox() : xy1({0,0}), xy2({0,0}), objectType(OBJECTTYPES::NA), distanceFromCamera(100000)
    {}
    BoundingBox(XYPoint<int> _xy1, XYPoint<int> _xy2, ObjectType _objectType, double distance)
    : xy1(_xy1), xy2(_xy2), objectType(_objectType), distanceFromCamera(distance)
    {}
    
    BoundingBox(const BoundingBox& bbox) 
    : xy1(bbox.xy1), xy2(bbox.xy2), objectType(bbox.objectType), distanceFromCamera(bbox.distanceFromCamera)
    {}

    BoundingBox& operator=(const BoundingBox& bbox) 
    { 
        if(this != &bbox) {
            xy1 = bbox.xy1;
            xy2 = bbox.xy2;
            objectType = bbox.objectType;
            distanceFromCamera = bbox.distanceFromCamera;
        }

        return *this;
    }

    //override operator=(const BoundingBox& bbox) 
    //: xy1(bbox.xy1), xy2(bbox.xy2), objectType(bbox.objectType) 
    //{}


>>>>>>> 2424c7d5a04b32b7fa621708b81a638598d6cbd7
    bool compare(const BoundingBox& a, const BoundingBox& b) const {
        return a.xy1.getX() < b.xy1.getX();
    }

    bool operator<(const BoundingBox& a) const {
        return xy1.getX() < a.xy1.getX();
    }

    XYPoint<int> getCenterXY() {
        int xpoint = xy1.getX() + ((xy2.getX() - xy1.getX()) / 2);
        int ypoint = xy1.getY() + ((xy2.getY() - xy1.getY()) / 2);
        return XYPoint<int>(xpoint, ypoint);
    }   
<<<<<<< HEAD
=======

    XYPoint<int> getXY1() { return xy1; }
    XYPoint<int> getXY2() { return xy2; }

    ObjectType getObjectType() { return objectType; }

    double getDistanceFromCamera() { return distanceFromCamera; }
    
>>>>>>> 2424c7d5a04b32b7fa621708b81a638598d6cbd7
private:
            
    XYPoint<int> xy1;
    XYPoint<int> xy2;
    ObjectType objectType;
<<<<<<< HEAD

=======
    double distanceFromCamera;
>>>>>>> 2424c7d5a04b32b7fa621708b81a638598d6cbd7
};

struct BoundingBoxCompare {
    bool operator()(const BoundingBox& a, const BoundingBox& b) const {
<<<<<<< HEAD
        //return a.xy1.getX() < b.xy1.getX();
=======
>>>>>>> 2424c7d5a04b32b7fa621708b81a638598d6cbd7
        return a < b;
    }
};

#endif