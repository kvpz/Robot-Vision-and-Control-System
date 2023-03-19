#ifndef TASK_HPP
#define TASK_HPP
#include <iostream>
#include "waypoint.hpp"
#include "enums/tasktype.hpp"
#include "enums/taskstatus.hpp"
#include "enums/robotState.hpp"
#include "map.hpp"
#include "navigator.hpp"

class Task
{
public:
  Task();
  Task(TaskType ttype);

  virtual void notStarted(Map* map, Navigator* navigator, RobotState& robotState);
  virtual void inProgress(Map* map, Navigator* navigator, RobotState& robotState);
  virtual void suspended();
  virtual void complete();

  // setters
  void setStatus(Status s);

  void setEndpoint(double x, double y, double orientation);

  // getters
  Status getStatus() const;
  Waypoint getDestination() const;
  TaskType getTaskType() const;
  inline std::string getName();
  double getEndpointOrientation();

  static void printTaskInfo(Task& task);
  
protected:
  Waypoint destination;

private:
  double endpointOrientation; // angle
  double expected_duration;
  Status status;
  TaskType taskType;
};

#endif
