#ifndef TASK_HPP
#define TASK_HPP

// go to 

enum Status {
	     NOTSTARTED, COMPLETE, INPROGRESS
};

class Task
{
public:
  Task(){ Status = INPROGRESS; }

  void setDestination(double x, double y)
  {
    destination.setX(x);
    destination.setY(y);
  }

  bool getStatus()
  {
    if(status == INPROGRESS || status == NOTSTARTED)
      return false;
    else
      return true;
  }
  
  class Behavior {
    
  };
  
private:
  size_t priority;
  Waypoint destination;
  
  double expected_duration;
  Status status;
  
};

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
