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

    virtual void notStarted(Map* map, Navigator* navigator, RobotState& robotState) override;

    virtual void inProgress(Map* map, Navigator* navigator, RobotState& robotState) override;
      
    virtual void suspended() override;

    virtual void complete() override;

private:
    RobotState robotState; // ex. MOVE_BACKWARD
    bool correcting_orientation = false;

};

#endif