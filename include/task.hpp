#ifndef TASK_HPP
#define TASK_HPP

// go to 

class Task
{
public:
  Task(){}

  class Behavior {
    
  };
  
private:
  size_t priority;
  Waypoint destination;
  
  double expected_duration;
  std::string status;
  
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
