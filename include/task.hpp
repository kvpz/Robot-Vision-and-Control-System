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
#include "settings.hpp"
#include "visiondata.hpp"

class Task
{
public:
  Task();
  Task(TaskType ttype, unsigned int priority);

  // task state functions
  virtual void notStarted(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          std::shared_ptr<VisionData> visionData,
                          RobotState& nextRobotState);

  virtual void inProgress(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          std::shared_ptr<VisionData> visionData,
                          RobotState& nextRobotState);

  virtual void suspended(std::shared_ptr<Map> map, 
                         std::shared_ptr<Navigator> navigator, 
                         std::shared_ptr<VisionData> visionData,
                         RobotState& nextRobotState, 
                         TaskType& nextTaskType);

  virtual void complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState,
                        TaskType& nextTaskType);

  // setters
  void setStatus(TaskStatus s);
  void setReadyForDeletion(bool v) { readyToBeDeleted = v; }

  // getters
  TaskStatus getStatus() const;
  TaskType getTaskType() const;
  std::string getName();
  unsigned int getPriority() { return priority_; }
  bool isReadyForDeletion() { return readyToBeDeleted; }

  // debug functions
  void printTaskInfo(Task& task);
  virtual void printTaskInfo();
  
protected:
  TaskType taskType;
  TaskStatus status;

private:
  // task management data
  double expected_duration;
  unsigned int priority_;
  unsigned int id;

  bool readyToBeDeleted;
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