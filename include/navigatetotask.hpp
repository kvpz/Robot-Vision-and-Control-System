#ifndef NAVIGATETASK_HPP
#define NAVIGATETASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"

#define DEBUG_NAVIGATETOTASK false

class NavigateToTask : public Task
{
public:
    NavigateToTask();

    virtual void notStarted(Map* map, Navigator* navigator, RobotState& robotState) override;

    virtual void inProgress(Map* map, Navigator* navigator, RobotState& robotState) override;

    virtual void suspended(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    /*
        When a robot has completed traveling to its endpoint, it then needs to 
        correct its orientation so that it matches the orientation required
        by the endpoint pose.
    */
    virtual void complete(Map* map, Navigator* navigator, RobotState& nextRobotState, TaskType& nextTaskType) override;

    //travelTaskSuspendedState(task);
    //travelTaskCompleteState(task);

    inline double getEndpointDesiredOrientation() { return endpointDesiredOrientation; }

private:
    RobotState robotState; // ex. MOVE_BACKWARD
    RobotPoseToWaypoint rPoseToWaypoint;
    bool isRobotAtEndpoint;
    Waypoint endpoint;
    double endpointDesiredOrientation; // angle
    bool isEndpointOrientationRequired;

    void EndpointPoseCorrection(Map* map, Navigator* navigator, RobotState& nextRobotState) ;

};

#endif