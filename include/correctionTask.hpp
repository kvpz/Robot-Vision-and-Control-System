#ifndef CORRECTIONTASK_HPP
#define CORRECTIONTASK_HPP
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "map.hpp"

//class Robot;

class CorrectionTask : public Task
{
public:
    CorrectionTask();

    virtual void notStarted(Map* map, Navigator* navigator, RobotState& robotState) override;

    virtual void inProgress(Map* map, Navigator* navigator, RobotState& robotState) override;

    virtual void suspended() override;

    virtual void complete() override;

private:
    RobotState robotState;
    double angleToDestTolerance = 10.0;
    bool correcting_position = false;        

};

#endif