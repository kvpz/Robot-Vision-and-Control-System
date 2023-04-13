#ifndef OBJECTS_HPP
#define OBJECTS_HPP
 
enum class OBJECTTYPES : int {
    YELLOWDUCK, 
    PINKDUCK,
    WHITEPEDESTAL,
    GREENPEDESTAL,
    REDPEDESTAL,
    NA
};

struct ObjectType {
public:
    ObjectType() : otype(OBJECTTYPES::NA)
    {}

    ObjectType(OBJECTTYPES type) {
      otype = type;
    }

    ObjectType(std::string otypestr) {
        if(otypestr == "duck") 
          otype = OBJECTTYPES::YELLOWDUCK;
        else if(otypestr == "green cylinder")
          otype = OBJECTTYPES::GREENPEDESTAL;
        else if(otypestr == "white cylinder")
          otype = OBJECTTYPES::WHITEPEDESTAL;
        else if(otypestr == "red cylinder") 
          otype = OBJECTTYPES::REDPEDESTAL;
        else 
          otype = OBJECTTYPES::NA;
    }

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

    bool operator<(const ObjectType& o) const {
        return otype < o.otype;
    }

    bool operator==(const ObjectType& o) const {
        return otype == o.otype;
    }

/*
    inline std::ostream& operator<<(std::ostream& os, const ObjectType& obj) const {
        os << obj.toString();
        return os;
    }
*/
    OBJECTTYPES getObjectType() const { return otype; }

    std::string toString() const
    {
        switch(otype) {
          case OBJECTTYPES::YELLOWDUCK:
            return "duck";
          case OBJECTTYPES::GREENPEDESTAL:
            return "green cylinder";
          case OBJECTTYPES::WHITEPEDESTAL:
            return "white cylinder";
          case OBJECTTYPES::REDPEDESTAL:
            return "red cylinder";
          default:
            return "error";
        }
    }

    void setObjectType(OBJECTTYPES ot) {
        otype = ot;
    }
    

private:
    OBJECTTYPES otype;
};
  
inline std::ostream& operator<<(std::ostream& os, const ObjectType& obj) {
    os << obj.toString();
    return os;
}
  
#endif
