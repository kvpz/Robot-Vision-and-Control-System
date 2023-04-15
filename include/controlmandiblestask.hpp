#ifndef CONTROLMANDIBLESTASK_HPP
#define CONTROLMANDIBLESTASK_HPP

#include "task.hpp"
#include "navigator.hpp"
#include "map.hpp"
#include "settings.hpp"
#include "xypoint.hpp"
#include "enums/mandibles.hpp"

class ControlMandiblesTask : public Task
{
public:
    ControlMandiblesTask(MandibleState desiredLeftState, 
                         MandibleState desiredRightState,
                         MandibleState currentLeftMandibleState,
                         MandibleState currentRightMandibleState,
                         XYPoint<double> xy, 
                         bool endpoint_orientation_requireds,
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
    MandibleState desiredLeftMandibleState;
    MandibleState desiredRightMandibleState;
    MandibleState currentLeftMandibleState;
    MandibleState currentRightMandibleState;

    // conditions for opening mandibles
    XYPoint<double> actionPoint;
    double actionPointOrientation;
    double actionPointProximityTolerance;
    
    bool inActionState;

    unsigned int actionStateSteps;

    bool isEndpointOrientationRequired;
    double desiredEndpointOrientation;
};

#endif