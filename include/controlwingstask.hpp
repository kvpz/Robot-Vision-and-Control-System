#ifndef CONTROLWINGSTASK_HPP
#define CONTROLWINGSTASK_HPP

#include "task.hpp"
#include "navigator.hpp"
#include "map.hpp"
#include "settings.hpp"
#include "xypoint.hpp"
#include "enums/wings.hpp"


class ControlWingsTask : public Task
{
public:
    ControlWingsTask(WingState desiredLeftState, 
                         WingState desiredRightState,
                         XYPoint<double> xy,
                         bool endpoint_orientation_required,
                         double endpointOrientation,
                         double actionPointProximityTolerance);

    virtual void notStarted(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            std::shared_ptr<VisionData> visionData,
                            RobotState& nextRobotState) override;

    virtual void inProgress(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            std::shared_ptr<VisionData> visionData,
                            RobotState& nextRobotState) override;

    virtual void suspended(std::shared_ptr<Map> map, 
                           std::shared_ptr<Navigator> navigator, 
                           std::shared_ptr<VisionData> visionData,
                           RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          std::shared_ptr<VisionData> visionData,
                          RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void printTaskInfo() override; //std::string taskStateName);

private:
    // mandibles to open
    WingState desiredLeftWingState;
    WingState desiredRightWingState;
    WingState currentLeftWingState;
    WingState currentRightWingState;

    // conditions for opening mandibles
    XYPoint<double> actionPoint;
    double actionPointProximityTolerance;
    
    bool inActionState;

    unsigned int actionStateSteps;

    bool isEndpointOrientationRequired;
    double desiredEndpointOrientation;
};

#endif