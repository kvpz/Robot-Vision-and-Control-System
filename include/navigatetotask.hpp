#ifndef NAVIGATETASK_HPP
#define NAVIGATETASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"

#define DEBUG_NAVIGATETOTASK true

class NavigateToTask : public Task
{
public:
    NavigateToTask(){}
    NavigateToTask(double endpointOrientation, bool endpointOrientationRequirement);

    virtual void notStarted(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void inProgress(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) override;

    virtual void suspended(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    /*
        When a robot has completed traveling to its endpoint, it then needs to 
        correct its orientation so that it matches the orientation required
        by the endpoint pose.
    */
    virtual void complete(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    //travelTaskSuspendedState(task);
    //travelTaskCompleteState(task);

    inline double getEndpointDesiredOrientation() { return endpointDesiredOrientation; }

private:

    bool isRobotAtEndpoint;
    Waypoint endpoint;
    double endpointDesiredOrientation; // angle
    bool isEndpointOrientationRequired;

    void EndpointPoseCorrection(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) ;

};

#endif