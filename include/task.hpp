#ifndef TASK_HPP
#define TASK_HPP
#include <iostream>
#include "waypoint.hpp"
#include "enums/tasktype.hpp"
#include "enums/taskstatus.hpp"
#include "enums/robotState.hpp"
#include "map.hpp"
#include "navigator.hpp"

#define DEBUG_TASK true

class Task
{
public:
  Task();
  Task(TaskType ttype);

  virtual void notStarted(Map* map, Navigator* navigator, RobotState& nextRobotState);
  virtual void inProgress(Map* map, Navigator* navigator, RobotState& nextRobotState);
  virtual void suspended(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType);
  virtual void complete(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType);

  // setters
  void setStatus(Status s);
  void setEndpoint(double x, double y, double orientation);

  // getters
  Status getStatus() const;
  Waypoint getDestination() const;
  TaskType getTaskType() const;
  inline std::string getName();
  inline double getEndpointDesiredOrientation() { return -1; }

  static void printTaskInfo(Task& task);
  
protected:
  Waypoint destination;
  TaskType taskType;

private:
  double expected_duration;
  Status status;
};

#endif
