#ifndef ROBOTORIENTATIONATENDPOINT_HPP
#define ROBOTORIENTATIONATENDPOINT_HPP

enum RobotOrientationAtEndpoint
{
    ORIENTED,
    OFF_TO_LEFT,
    OFF_TO_RIGHT,
    NOTORIENTED // "dumb" state. robot would keep rotating in same direction until orientation achieved
};

#endif