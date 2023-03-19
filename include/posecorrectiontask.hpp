#ifndef POSECORRECTIONTASK_HPP
#define POSECORRECTIONTASK_HPP
#include "task.hpp"
#include "enums/robotOrientationAtEndpoint.hpp"
#include "navigator.hpp"
#include "map.hpp"

class PoseCorrectionTask : public Task
{
public:
    PoseCorrectionTask();

    virtual void notStarted(Map* map, Navigator* navigator, RobotState& nextRobotState) override;

    virtual void inProgress(Map* map, Navigator* navigator, RobotState& nextRobotState) override;
      
    virtual void suspended(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

private:
    //RobotState robotState; // ex. MOVE_BACKWARD
    bool correcting_orientation = false;

};

#endif