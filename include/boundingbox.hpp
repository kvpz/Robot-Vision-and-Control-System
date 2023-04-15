#ifndef BOUNDINGBOX_HPP
#define BOUNDINGBOX_HPP

class BoundingBox {
public:
    BoundingBox(XYPoint<int> _xy1, XYPoint<int> _xy2, ObjectType _objectType)
    : xy1(_xy1), xy2(_xy2), objectType(_objectType)
    {}
    
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
private:
            
    XYPoint<int> xy1;
    XYPoint<int> xy2;
    ObjectType objectType;

};

struct BoundingBoxCompare {
    bool operator()(const BoundingBox& a, const BoundingBox& b) const {
        //return a.xy1.getX() < b.xy1.getX();
        return a < b;
    }
};

#endif