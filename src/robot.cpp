#include "robot.hpp"

Robot::Robot(double xpos, double ypos, double orientation) 
    : state(STOP), robotPoseToWaypoint(NONE)
  //: map(std::make_shared<Map>()), navigator(std::make_unique<Navigator>()), taskManager(std::make_unique<TaskManager>())
{ 
    comport = std::make_unique<Comms>("/dev/ttyACM0");
    taskManager = std::make_shared<TaskManager>();
    navigator = std::make_unique<Navigator>();
    map = std::make_unique<Map>();
    map->setRobotCurrentCoordinate(xpos, ypos);
    map->setRobotOrientation(orientation);

    robotPoseToWaypoint = NONE;
    robotOrientationAtEndpoint = NOTORIENTED;

    std::cout << "======== Robot::Robot ========\n";
    std::cout << "taskManager address: " << &(*taskManager) << "\n";
    std::cout << "==============================\n" << std::endl;
}

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

void Robot::move_forward()  { comport->send_command("F"); }
void Robot::move_backward() { comport->send_command("B"); }
void Robot::move_left()     { comport->send_command("L"); }
void Robot::move_right()    { comport->send_command("R"); }
void Robot::rotate_CW()     { comport->send_command("C"); }
void Robot::rotate_CCW()    { comport->send_command("Z"); }
void Robot::stop()          { comport->send_command("S"); }

// getters (inlined)
RobotState Robot::getState() const { return state; }

double Robot::getX() const { return map->getNextDestinationXY().getX(); }
double Robot::getY() const { return map->getNextDestinationXY().getY(); }
double Robot::getOrientation() const { return map->getRobotOrientation(); }

bool Robot::hasTasks() { return taskManager->hasTasks(); }

std::shared_ptr<TaskManager> Robot::getTaskManager() { return taskManager; }
std::shared_ptr<Map> Robot::getMap() { return map; }
std::shared_ptr<Navigator> Robot::getNavigator() { return navigator; }


// setters (inlined)
void Robot::setCurrentXY(double x, double y) 
{
  map->setRobotCurrentCoordinate(x,y);
}

void Robot::setOrientation(double o)
{
  map->setRobotOrientation(o);
}
    
/*
  Robot sends data about itself to the task manager. 
  The task manager then executes the current task. 
  A task will update the state of the robot.
*/
void Robot::executeCurrentTask()
{
  if(ROBOTDEBUG) {
    std::cout << "====== Robot::executeCurrentTask =======" << std::endl;
  }

  RobotState nextRobotState;
  //TODO: nextRobotState = taskManager->executeCurrentTask(map, navigator);
  taskManager->executeCurrentTask(map, navigator, nextRobotState);

  // change robot state if it is different from current state
  if(state != nextRobotState) {
      state = nextRobotState;
      run(); // alter robot state if it needs to be in a different
  }
}

void Robot::printStatus() 
{
    std::cout << "\n====== Robot Status ======\n";
    std::cout << "State: " << RobotStateToString(state) << "\n";
    std::cout << "current location: ("
              << map->RobotX() << ", "
              << map->RobotY() << ")\n";
    std::cout << "current orientation (yaw): " << map->getRobotOrientation() << "\n";
    std::cout << "next destination: (" 
              << map->getNextDestinationXY().getX() << ", "
              << map->getNextDestinationXY().getY() << ")\n";
    std::cout << "==========================\n";
    std::cout << std::endl;
}