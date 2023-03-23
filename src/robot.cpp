#include "robot.hpp"

Robot::Robot(double xpos, double ypos) 
    : state(STOP)
  //: map(std::make_unique<Map>()), navigator(std::make_unique<Navigator>()), taskManager(std::make_unique<TaskManager>())
{ 
    comport = new Comms("/dev/ttyACM0");
    robotPoseToWaypoint = NONE;
    state = STOP;
    taskManager = std::make_unique<TaskManager>();
    navigator = std::make_unique<Navigator>();
    map = std::make_unique<Map>();
    map->setRobotCurrentCoordinate(xpos, ypos);
}

//void run();
void Robot::run()
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

void Robot::move_forward() { comport->send_command("F"); }
void Robot::move_backward() { comport->send_command("B"); }
void Robot::move_left() { comport->send_command("L"); }
void Robot::move_right() { comport->send_command("R"); }
void Robot::rotate_CW() { comport->send_command("C"); }
void Robot::rotate_CCW() { comport->send_command("Z"); }
void Robot::stop() { comport->send_command("S"); }

void Robot::printStatus() 
{
    std::cout << "\n====== Robot Status ======\n";
    std::cout << "State: " << RobotStateToString(state) << "\n";
    std::cout << "current location: ("
            << map->RobotX() << ", "
            << map->RobotY() << ")\n";
    std::cout << "current orientation (yaw): " << map->getRobotOrientation() << "\n";
    std::cout << "robot pose relative to waypoint: " << printRobotPoseToWaypoint(robotPoseToWaypoint) << "\n";
    std::cout << "==========================\n";
    std::cout << std::endl;
}

// getters
double Robot::getX() const { return map->RobotX(); }
double Robot::getY() const { return map->RobotY(); }
RobotState getState() const { return state; }
double getOrientation() const { return map->getRobotOrientation(); }
double getAngleToDestination() const { return navigator->getAngleToDestination(); }
std::unique_ptr<TaskManager> getTaskManager() { return std::move(taskManager); }
bool hasTasks() { return taskManager->hasTasks(); }

std::unique_ptr<Map> getMap() { return map; }
std::unique_ptr<Navigator> getNavigator() { return navigator; }

// setters
void setTravelDirection(RobotState travDir) { state = travDir; } //travelDirection = travDir; }

void updateRobotState(RobotState nextRobotState)
{
    if(state != nextRobotState) {
        state = nextRobotState;
        run();
    }
}

void setCurrentXY(double x, double y) 
{
  map->setRobotCurrentCoordinate(x,y);
}

void setOrientation(double o)
{
  map->setRobotOrientation(o);
}

/*
  Robot sends data about itself to the task manager. 
  The task manager then executes the current task. 
  A task will update the state of the robot.
*/
void executeCurrentTask()
{
  //state = taskManager->executeCurrentTask(map, navigator);
  taskManager->executeCurrentTask(map, navigator, nextRobotState);
  updateRobotState(nextRobotState);
}

/*
Robot::Robot() : state(STOP), robotPoseToWaypoint(NONE), robotOrientationAtEndpoint(NOTORIENTED), nearEndpoint(false)
{ 
    comport = new Comms("/dev/ttyACM0");
    robotPoseToWaypoint = NONE;
    state = STOP;
}
*/
/*
void Robot::run()
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

double Robot::getRobotAngleToPoseOrientation(const Robot& robot, double endpointOrientation) const
{
  return angleToEndpointOrientation(robot.getOrientation(), endpointOrientation);
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
    bool isYapproxnear = approximately(robotY, destY, 2.0);
    bool isXapproxnear = approximately(robotX, destX, 2.0);
    double angleToDestTolerance = 10.0;

    angleToDestination = getRobotAngleToPoint(*this, destX, destY);

    if(isYapproxnear && isXapproxnear) {
        // near the waypoint
        result = NEAR;
    }
    else if (angleToDestination < angleToDestTolerance && angleToDestination > -1.0*angleToDestTolerance
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
        std::cout << "(isRobotOnPath) angle to dest: " << angleToDestination << "\n";
        std::cout << "=============================\n" << std::endl;
    }

    robotPoseToWaypoint = result;
    return result;
}

RobotOrientationAtEndpoint Robot::isRobotOriented(double robotOrientation, double endpointOrientation)
{
  RobotOrientationAtEndpoint result = ORIENTED;
  double tolerance = 5.0;
  bool isRobotApproximatelyOriented = false;
  
  // robot orientation minus endpoint orientation
  angleToDestination = getRobotToEndpointSlopeAngle(*this, endpointOrientation);
  isRobotApproximatelyOriented = std::fabs(angleToDestination) > tolerance ? false : true;
  
  if(angleToDestination < 0.0 && isRobotApproximatelyOriented) {
    // rotate CCW if absolute value of difference is less than 180
    if(std::fabs(angleToDestination) < 180.0 && std::fabs(angleToDestination) > tolerance)
      result = OFF_TO_RIGHT;
    else
      result = OFF_TO_LEFT;
  }
  else if(angleToDestination > 0.0 && std::fabs(angleToDestination) > tolerance) {
    // rotate CW if
    if(std::fabs(angleToDestination) < 180.0)
      result = OFF_TO_LEFT;
    else
      result = OFF_TO_RIGHT;
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
*/