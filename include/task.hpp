#ifndef TASK_HPP
#define TASK_HPP
#include <iostream>
#include "waypoints.hpp"

namespace ROBOTASKS 
{
  enum Status {
        NOTSTARTED, COMPLETE, INPROGRESS, SUSPENDED
  };

  enum TaskType {
          TRAVEL, CHIPDROP, RECYCLE, GRASP, STACKPED,
          CORRECTPATH, ROTATE
  };

  static std::string statusToString(Status status) 
  {
    switch(status) {
      case NOTSTARTED:
        return "NOTSTARTED";
      case COMPLETE:
        return "COMPLETE";
      case INPROGRESS:
        return "INPROGRESS";
      case SUSPENDED:
        return "SUSPENDED";
      default:
        return "ERROR";
    }
  }

  class Task
  {
  public:
    Task() {};
    Task(TaskType ttype, std::string name)
      : status(NOTSTARTED), taskType(ttype), nameid(name)
    {
    }

    // setters
    void setDestination(double x, double y)
    {
      destination.setX(x);
      destination.setY(y);
    }

    void setStatus(Status s)
    {
      status = s;
    }

    void setDesiredRobotYawPose(double y)
    {
      desiredRobotYawPose = y;
    }
    
    // getters
    Status getStatus() const { return status; }
    Waypoint getDestination() const { return destination; } 
    TaskType getTaskType() const { return taskType; }
    inline std::string getName() { return nameid; }
    inline double getDesiredRobotYawPose() { return desiredRobotYawPose; }
    
  private:
    Waypoint destination;
    double desiredRobotYawPose; // angle
    double expected_duration;
    Status status;
    TaskType taskType;
    std::string nameid;
  };
}
#endif

/*
  Task A:
  Behavior of robot at destination

  This class should receive data for task status updates. 
  It can receive the position of the robot relative to the task destination. 
  [Maybe] The task can mark itself complete.

  A Task object would be a Robot's member data. 
  The task objecsts can be stored in a generic c++ data structure. 
  The task scheduler data structures of Task objects will track the completion of tasks. 

  The robot can have a stack of completed/incompleted tasks. 
  Task tracking and timestamping are important because they can be used to verify behaviors.
  
  

 */
