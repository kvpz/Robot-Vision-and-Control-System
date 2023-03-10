#include "robot.hpp"

Robot::Robot()
{ 
    comport = new Comms("/dev/ttyACM0");
    robotPoseToWaypoint = NONE;
}

void Robot::run(Task& task) {
    switch (state) {
    case MOVE_FORWARD:
      move_forward(); 
      break;
    case MOVE_LEFT:
      //move_left();
      break;
    case MOVE_RIGHT:

      break;
    case ROTATE_CW:
      rotate_CW(task.getDesiredRobotYawPose()); 
      break;
    case ROTATE_CCW:
      rotate_CCW(task.getDesiredRobotYawPose()); 
      break;
    case MOVE_BACKWARD:

      break;
    case STOP:
      stop();
      break;
    }
  }

double Robot::robotAngleToPoint(const Robot& robot, double x, double y) const
{
    return angleToPoint(robot.getX(), robot.getY(), x, y, robot.getOrientation());
}

void Robot::move_forward()
{
    comport->send_command("F");
}

void Robot::move_forward(double distance, double robot_speed_per_sec)
{
    if(ROBOTDEBUG) std::cout << "move forward" << std::endl;
    comport->send_command("F");
    //std::cout << "Sleep duration: " << (long)(distance / robot_speed_per_sec * 1000.0) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds((long)(distance / robot_speed_per_sec * 1000.0)));
    stop();
}

void Robot::move_backward(double distance, double robot_speed_per_sec)
{
    if(ROBOTDEBUG) std::cout << "move backward" << std::endl;
    comport->send_command("B");
    //std::cout << "Sleep duration: " << (long)(distance / robot_speed_per_sec * 1000.0) << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds((long)(distance / robot_speed_per_sec * 1000.0)));
    stop();
}

void Robot::rotate_CW(double degrees) //, double robot_speed)
{
    if(ROBOTDEBUG) std::cout << "rotate_CW ( " << degrees << " )" << std::endl;
    comport->send_command("C");
    //std::cout << "Sleep duration: " << (long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees) << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds((long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees)));
}

void Robot::rotate_CCW(double degrees) //, double robot_speed)
{
    if(ROBOTDEBUG) std::cout << "rotate_CCW ( " << degrees << " )" << std::endl;
    comport->send_command("Z");
    //std::cout << "Sleep duration: " << (long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees) << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds((long)((ROBOT_360_ROTATE_TIME_MILLISEC / 360.0) * degrees)));
}

void Robot::stop()
{
    comport->send_command("S");
}

void Robot::go_to_destination(double x_robot, double y_robot, double x_destination, double y_destination, double robot_current_angle)
{   
    double x_diff = x_destination - x_robot;
    double y_diff = y_destination - y_robot;
    double beta = 0.0;
    double theta = 0.0;
    double distance = 0.0;

    beta = atan(y_diff / x_diff) * 180.0 / (M_PI);

    if(x_diff > 0.0 && y_diff > 0.0){
        // first quadrant

    }
    else if(x_diff < 0.0 && y_diff > 0.0) {
        // second quadrant
        beta = beta + 180.0;
    }
    else if(x_diff < 0.0 && y_diff < 0.0) {
        // third quadrant
        beta = beta + 180.0;
    }
    else {
        // fourth quadrant
        beta = beta + 270.0;
    }

    theta = beta - robot_current_angle;
    distance = sqrt(x_diff * x_diff + y_diff * y_diff);
    std::cout << "robot current angle:  " << robot_current_angle << std::endl;
    std::cout << "distance: " << distance << std::endl;
    std::cout << "x2 - x1: " << x_diff << std::endl;
    std::cout << "y2 - y1: " << y_diff << std::endl;
    std::cout << "beta: " << beta << std::endl;
    std::cout << "theta: " << theta << std::endl;

    if(theta < 0.0) {
        // go clockwise
        rotate_CW(theta); //, ROBOT_360_ROTATE_TIME_MILLISEC);
    }
    else {
        // go counterclockwise
        rotate_CCW(theta); //, ROBOT_360_ROTATE_TIME_MILLISEC);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    move_forward(distance, ROBOT_SPEED_CM_PER_SEC);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

RobotPoseToWaypoint Robot::isRobotOnPath(double robotX, double robotY, double destX, double destY)
{
    RobotPoseToWaypoint result = ON_PATH;
    // check if robot position (x,y) approximately near destination
    bool isYapproxnear = approximately(robotY, destY, 3.0, false);
    bool isXapproxnear = approximately(robotX, destX, 3.0, false);
    double angleToDestTolerance = 10.0;

    angleToDest = robotAngleToPoint(*this, destX, destY);

    if(isYapproxnear && isXapproxnear) {
        // near the waypoint
        result = NEAR;
    }
    else if (angleToDest < angleToDestTolerance && angleToDest > -1.0*angleToDestTolerance
            && (!(robotX < (destX - 2.5)) || !(robotX > (destX + 2.5)))) { // robotY > destY && robotX == destX
        // detect if drifting from path
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

RobotPoseToWaypoint Robot::robotPositionRelativeToWaypoint(double robotX, double robotY, double destX, double destY)
{
    RobotPoseToWaypoint result = ON_PATH;
    // check if robot position (x,y) approximately near destination
    bool isYapproxnear = approximately(robotY, destY, 1.5, false);
    bool isXapproxnear = approximately(robotX, destX, 1.5, false);

    angleToDest = robotAngleToPoint(*this, destX, destY);

    if(angleToDest < (90.0 - 1.0)) {
        
    }
    else if (angleToDest > (90.0 + 1.0)) {
        
    }

    if(isYapproxnear && isXapproxnear) {
        // near the waypoint
        result = NEAR;
    }
    else if (angleToDest < 1.0) { // robotY > destY && robotX == destX
        // detect if drifting from path
        result = ON_PATH;
    }
    else if(robotY < destY && robotX < destX) {
        // before waypoint and off to the left
        result = BEFORE_LEFT;
    }
    else if(robotY < destY && robotX > destX) {
        // before waypoint and off to the right
        result = BEFORE_RIGHT;
    }
    else if(robotY > destY && robotX < destX) {
        // ahead of waypoint and to the left
        result = AFTER_LEFT;
    }
    else if(robotY > destY && robotX > destX) {
        // ahead of waypoint and to the right
        result = AFTER_RIGHT;
    }

    if(ROBOTDEBUG) {
        std::cout << "\n====== robotPostitionRelativeToWaypoint ======\n";
        std::cout << "result: " << printRobotPoseToWaypoint(result) << "\n";
        std::cout << "angle to dest: " << angleToDest << std::endl;
        std::cout << "==============================================\n" << std::endl;
}

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