#ifndef ROBOT_HPP
#define ROBOT_HPP
#include "waypoints.hpp"
#include "taskmanager.hpp"
//#include "task.hpp"
#include "comms.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"

using namespace std;

#define ROBOTDEBUG true

class TaskManager;

class Robot {
public:
    Robot() 
    { 
        comport = new Comms("/dev/ttyACM0");
        robotPoseToWaypoint = NONE;
        state = STOP;
        taskManager = new TaskManager();
    }

    //void run();
    void run()
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

    double getRobotAngleToPoint(double x, double y) const 
    {
          return angleToPoint(getX(), getY(), x, y, currentOrientation);
    }

    double getRobotAngleToPoseOrientation(double endpointOrientation) const 
    {
        return angleToEndpointOrientation(currentOrientation, endpointOrientation);
    }

    void move_forward() { comport->send_command("F"); }
    void move_backward() { comport->send_command("B"); }
    void move_left() { comport->send_command("L"); }
    void move_right() { comport->send_command("R"); }
    void rotate_CW() { comport->send_command("C"); }
    void rotate_CCW() { comport->send_command("Z"); }
    void stop() { comport->send_command("S"); }

    RobotPoseToWaypoint isRobotOnPath(double robotX, double robotY, double destX, double destY) 
    {
        RobotPoseToWaypoint result = ON_PATH;
        // check if robot position (x,y) approximately near destination
        bool isYapproxnear = approximately(robotY, destY, 2.0);
        bool isXapproxnear = approximately(robotX, destX, 2.0);
        double angleToDestTolerance = 10.0;

        angleToDestination = getRobotAngleToPoint(destX, destY);

        if(isYapproxnear && isXapproxnear) {
            // near the waypoint
            result = NEAR;
        }
        else if (angleToDestination < angleToDestTolerance && angleToDestination > -1.0*angleToDestTolerance
                && (!(robotX < (destX - 2.5)) || !(robotX > (destX + 2.5)))) 
        { // robotY > destY && robotX == destX
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

    RobotOrientationAtEndpoint isRobotOriented(double robotOrientation, double endpointOrientation) 
    {
        RobotOrientationAtEndpoint result = ORIENTED;
        double tolerance = 5.0;
        bool isRobotApproximatelyOriented = false;

        // robot orientation minus endpoint orientation
        angleToDestination = getRobotAngleToPoseOrientation(endpointOrientation);
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

    void printStatus() 
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

    // getters (inlined)
    inline double getX() const { return currentLocation.getX(); }
    inline double getY() const { return currentLocation.getY(); }
    inline RobotState getState() const { return state; }
    inline double getOrientation() const { return currentOrientation; }
    inline double getAngleToDestination() const { return angleToDestination; }
    inline bool isNearEndpoint() const { return nearEndpoint; }
    inline RobotState getTravelDirection() { return travelDirection; }

    // setters (inlined)
    void setTravelDirection(RobotState travDir) { travelDirection = travDir; }

    void setCurrentXY(double x, double y) 
    {
      currentLocation.setX(x);
      currentLocation.setY(y);
    }
    
    void setState(RobotState newState) 
    {
      state = newState;
    }

    void setOrientation(double o)
    {
      currentOrientation = o;
    }
    
    void setIsNearEndpoint(bool b) { nearEndpoint = b; }

    void executeCurrentTask()
    {
      taskManager->executeCurrentTask(this);
      updateRobotState(taskManager->getNextRobotState());
    }

    void updateRobotState(RobotState nextRobotState)
    {
        if(state != nextRobotState) {
            setState(nextRobotState);
            run();
        }
    }

    TaskManager& getTaskManager()
    {
      return *taskManager;
    }

    bool hasTasks() 
    {
      return taskManager->hasTasks();
    }

private:
    RobotState state = STOP;
    RobotState nextRobotState;
    Waypoint currentLocation;
    double currentOrientation; // (gyro) orientation
    double velocity;
    double currentAngle; // relative to starting position
    Comms* comport;
    RobotPoseToWaypoint robotPoseToWaypoint = NONE;
    RobotOrientationAtEndpoint robotOrientationAtEndpoint = NOTORIENTED;
    double angleToDestination;
    bool nearEndpoint = false;
    RobotState travelDirection;

    TaskManager* taskManager;
};


#endif
