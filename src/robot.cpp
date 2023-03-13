#include "robot.hpp"

Robot::Robot()
{ 
    comport = new Comms("/dev/ttyACM0");
    robotPoseToWaypoint = NONE;
    state = STOP;
}

void Robot::run() //Task& task) {
{
    switch (state) {
    case MOVE_FORWARD:
      move_forward(); 
      break;
    case MOVE_LEFT:
      move_left();
      break;
    case MOVE_RIGHT:
        move_right();
      break;
    case ROTATE_CW:
      rotate_CW(); 
      break;
    case ROTATE_CCW:
      rotate_CCW(); 
      break;
    case MOVE_BACKWARD:
      move_backward();
      break;
    case STOP:
      stop();
      break;
    }
  }

double Robot::getRobotAngleToPoint(const Robot& robot, double x, double y) const
{
    return angleToPoint(robot.getX(), robot.getY(), x, y, robot.getOrientation());
}

void Robot::move_forward()
{
    comport->send_command("F");
}

void Robot::stop()
{
    comport->send_command("S");
}

void Robot::rotate_CW()
{
    comport->send_command("C");
}

void Robot::rotate_CCW()
{
    comport->send_command("Z");
}

void Robot::move_backward()
{
    comport->send_command("B");
}

void Robot::move_left()
{
    comport->send_command("L");
}

void Robot::move_right()
{
    comport->send_command("R");
}

RobotPoseToWaypoint Robot::isRobotOnPath(double robotX, double robotY, double destX, double destY)
{
    RobotPoseToWaypoint result = ON_PATH;
    // check if robot position (x,y) approximately near destination
    bool isYapproxnear = approximately(robotY, destY, 2.0, false);
    bool isXapproxnear = approximately(robotX, destX, 2.0, false);
    double angleToDestTolerance = 5.0;

    angleToDest = getRobotAngleToPoint(*this, destX, destY);

    if(isYapproxnear && isXapproxnear) {
        // near the waypoint
        result = NEAR;
    }
    else if (angleToDest < angleToDestTolerance && angleToDest > -1.0*angleToDestTolerance
            && (!(robotX < (destX - 2.5)) || !(robotX > (destX + 2.5)))) { // robotY > destY && robotX == destX
        // detect if drifting from path
        std::cout << "\n(EXPERIMENTAL) condition\n" << std::endl;
        result = ON_PATH;
    }
    else {
        result = OFF_PATH;
    }    

    if(ROBOTDEBUG) {
        std::cout << "\n====== isRobotOnPath ======\n";
        std::cout << "result: " << printRobotPoseToWaypoint(result) << "\n";
        std::cout << "(isRobotOnPath) angle to dest: " << angleToDest << "\n";
        std::cout << "=============================\n" << std::endl;
    }

    robotPoseToWaypoint = result;
    return result;
}

void Robot::printStatus()
{
    std::cout << "\n====== Robot Status ======\n";
    std::cout << "State: " << RobotStateToString(state) << "\n";
    std::cout << "current location: ("
            << currentLocation.getX() << ", "
            << currentLocation.getY() << ")\n";
    std::cout << "current orientation (yaw): " << currentOrientation << "\n";
    std::cout << "robot pose relative to waypoint: " << printRobotPoseToWaypoint(robotPoseToWaypoint) << "\n";
    std::cout << "==========================\n";
    std::cout << std::endl;
}
