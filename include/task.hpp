#ifndef TASK_HPP
#define TASK_HPP
#include <iostream>
#include "waypoints.hpp"
#include "enums/tasktype.hpp"
#include "enums/taskstatus.hpp"

namespace ROBOTASKS 
{
  class Task
  {
  public:
    Task() {};
    Task(TaskType ttype)
      : status(NOTSTARTED), taskType(ttype)
    {
    }

    // setters
    void setStatus(Status s)
    {
      status = s;
    }

    void setEndpoint(double x, double y, double orientation)
    {
      destination.setX(x);
      destination.setY(y);
      endpointOrientation = orientation;
    }
    
    // getters
    Status getStatus() const { return status; }
    Waypoint getDestination() const { return destination; } 
    TaskType getTaskType() const { return taskType; }
    inline std::string getName() { return taskTypeToString(taskType); }
    inline double getEndpointOrientation() { return endpointOrientation; }

    static void printTaskInfo(ROBOTASKS::Task& task)
    {
        // print status of this type of task
        switch(task.getTaskType()) {
            case TRAVEL:
                std::cout << "\n====== Travel Task Updater ======\n";
                break;
            case CORRECTPATH:
                std::cout << "\n====== CorrectPath Task Updater ======\n";
                break;  
        }

        std::cout << "Task destination (X,Y): (" << task.getDestination().getX() 
                << ", " << task.getDestination().getY() << ")\n";
        std::cout << "Task status: " << statusToString(task.getStatus()) << "\n";
        std::cout << "Task name: " << task.getName() << "\n";
        std::cout << "=================================\n";
        std::cout << std::endl;
    }
    
  private:
    Waypoint destination;
    double endpointOrientation; // angle
    double expected_duration;
    Status status;
    TaskType taskType;
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
