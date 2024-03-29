#ifndef DROPCHIPTASK_HPP
#define DROPCHIPTASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"
#include "settings.hpp"

#define DROPCHIPTASK_PRIORITY 3

class DropChipTask : public Task
{
public:
    DropChipTask();
    DropChipTask(XYPoint<double> xy, 
                double endpointOrientation, 
                bool endpointOrientationRequirement);

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

    virtual void printTaskInfo() override;

private:
    // intelligence gathering data
    AttractionColors attractionColor;

    // payload management 
    unsigned int runtime;
    bool isDeployingPayload;
    inline static unsigned redDeploymentAttempts = 0;
    inline static unsigned greenDeploymentAttempts = 1;

    // payload location data
    XYPoint<double> payloadLocation;
    double endpointOrientation;
    bool endpointOrientationRequirement;

    // suspended state reasons
    TaskType newTaskRequest;
};

#endif
