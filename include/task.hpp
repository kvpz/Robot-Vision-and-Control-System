#ifndef TASK_HPP
#define TASK_HPP
#include <iostream>
#include <memory>
#include <optional>
#include "waypoint.hpp"
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
  virtual void notStarted(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState);
  virtual void inProgress(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState);
  virtual void suspended(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType);
  virtual void complete(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType);

  // setters
  void setStatus(Status s);
  void setEndpoint(std::unique_ptr<Map> map);
  void setEndpoint(double destx, double desty, double destOrientation);

  // getters
  Status getStatus() const;
  Waypoint getDestination() const;
  TaskType getTaskType() const;
  std::string getName();
  //std::optional<double> getEndpointDesiredOrientation() { return endpointDesiredOrientation; }
  double getEndpointDesiredOrientation() { return -1; }

  static void printTaskInfo(Task& task);
  
protected:
  Waypoint destination;
  TaskType taskType;

  /*
    Destination data is considered to be essential for every task, especially the following tasks:
    NavigateTo, PathCorrectionTask, PoseCorrectionTask (functions now part of NavigateTo)
  */
private:
  double expected_duration;
  Status status;
  double endpointDesiredOrientation;
};

#endif

/*
  TODO 1 (serial communication fault handling): 
    Add exception handling for read/write to comport functions. 
    Must expect there to be an issue 
*/