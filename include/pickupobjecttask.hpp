#ifndef PICKUPOBJECTTASK_HPP
#define PICKUPOBJECTTASK_HPP

#include "robot.hpp"
#include "task.hpp"
//#include "enums/robotPoseToWaypoint.hpp"
//#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"

#define DEBUG_PICKUPOBJECTTASK false

#define PICKUPOBJECTTASK_PRIORITY 3

class PickupObjectTask : public Task
{
public:
    PickupObjectTask();

    virtual void notStarted(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void inProgress(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void suspended(std::shared_ptr<Map> map, 
                           std::shared_ptr<Navigator> navigator, 
                           RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          RobotState& nextRobotState, TaskType& nextTaskType) override;

private:

    // payload location data
    XYPoint payloadLocation;
    double endpointOrientation;
    bool endpointOrientationRequirement;


};

#endif