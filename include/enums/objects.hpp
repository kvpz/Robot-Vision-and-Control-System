#ifndef OBJECTS_HPP
#define OBJECTS_HPP
 
enum class OBJECTTYPES {
    YELLOWDUCK, 
    PINKDUCK,
    WHITEPEDESTAL,
    GREENPEDESTAL,
    REDPEDESTAL
};

struct ObjectType {
public:
    ObjectType() = default;

    ObjectType(const ObjectType& o) 
        : otype(o.otype)
    {
    }

    // Move constructor
    ObjectType(ObjectType&& other) noexcept : otype(std::move(other.otype)) {}


    ObjectType& operator=(const ObjectType& o) 
    {
        if(this != &o) {
            otype = o.otype;
        }
        return *this;
    }

    ObjectType& operator=(const ObjectType&& o)
    {
        if(this != &o) {
            otype = std::move(o.otype);
        }
        return *this;
    }

    // Move assignment operator
    ObjectType& operator=(ObjectType&& other) noexcept {
        if (this != &other) {
            otype = std::move(other.otype);
        }
        return *this;
    }
private:
    OBJECTTYPES otype;
};

#endif
