#ifndef TASK_HPP
#define TASK_HPP
#include <iostream>
#include <memory>
#include <optional>
#include <mqueue.h>
#include <climits>
#include "navigator.hpp"
#include "xypoint.hpp"
#include "enums/tasktype.hpp"
#include "enums/taskstatus.hpp"
#include "enums/robotState.hpp"
#include "map.hpp"

#define DEBUG_TASK false 

class Task
{
public:
  Task();
  Task(TaskType ttype, unsigned int priority);

  // task state functions
  virtual void notStarted(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          RobotState& nextRobotState);

  virtual void inProgress(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          RobotState& nextRobotState);

  virtual void suspended(std::shared_ptr<Map> map, 
                         std::shared_ptr<Navigator> navigator, 
                         RobotState& nextRobotState, 
                         TaskType& nextTaskType);

  virtual void complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState,
                        TaskType& nextTaskType);

  // setters
  void setStatus(TaskStatus s);

  // getters
  TaskStatus getStatus() const;
  TaskType getTaskType() const;
  std::string getName();
  unsigned int getPriority() { return priority_; }

  // debug functions
  void printTaskInfo(Task& task);
  
protected:
  TaskType taskType;
  TaskStatus status;

private:
  // task management data
  double expected_duration;
  unsigned int priority_;
  unsigned int id;
};

#endif

/*
  TODO 1 (serial communication fault handling): 
    Add exception handling for read/write to comport functions. 
    Must expect there to be an issue 

      // endpoint should be set on the map separately
  // this does not belong here.
  // Map class should be in charge of storing the endpoints (destinations) and other points
  // This function may be good in NavigateTo task especially when intializing the instances
  // that will be stored in the task manager queue.
  //void setEndpoint(std::shared_ptr<Map> map);
  //void setEndpoint(double destx, double desty, double destOrientation);

*/