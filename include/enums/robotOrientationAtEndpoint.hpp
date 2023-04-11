#ifndef ROBOTORIENTATIONATENDPOINT_HPP
#define ROBOTORIENTATIONATENDPOINT_HPP

enum RobotOrientationAtEndpoint
{
    ORIENTED,
    OFF_TO_LEFT,
    OFF_TO_RIGHT,
    NOTORIENTED // "dumb" state. robot would keep rotating in same direction until orientation achieved
};

static std::string RobotOrientationAtEndpointToString(RobotOrientationAtEndpoint var)
{
    switch(var) {
        case ORIENTED:
            return "oriented";
            break;
        case OFF_TO_LEFT:
            return "off to left";
            break;
        case OFF_TO_RIGHT:
            return "off to right";
            break;
        case NOTORIENTED:
            return "not oriented";
            break;
        default:
            return "";
    }
}

#endif