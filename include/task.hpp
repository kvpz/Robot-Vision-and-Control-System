#ifndef TASK_HPP
#define TASK_HPP
#include <iostream>
#include <memory>
#include <optional>
#include "xypoint.hpp"
#include "enums/tasktype.hpp"
#include "enums/taskstatus.hpp"
#include "enums/robotState.hpp"
#include "map.hpp"
#include "navigator.hpp"

#define DEBUG_TASK false 

class Task
{
public:
  Task() : status(NOTSTARTED){}
  Task(TaskType ttype);

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

  void printTaskInfo(Task& task);
  
protected:
  TaskType taskType;

  /*
    Destination data is considered to be essential for every task, especially the following tasks:
    NavigateTo, PathCorrectionTask, PoseCorrectionTask (functions now part of NavigateTo)
  */
private:
  // task management data
  double expected_duration;
  TaskStatus status;
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