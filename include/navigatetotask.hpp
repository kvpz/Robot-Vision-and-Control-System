#ifndef NAVIGATETASK_HPP
#define NAVIGATETASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"

#define DEBUG_NAVIGATETOTASK false

#define ORIENTATION_RANGE_TOLERANCE 2.0

#define NAVIGATETOTTASK_PRIORITY 3

class NavigateToTask : public Task
{
public:
    NavigateToTask();
    NavigateToTask(double endpointOrientation, bool endpointOrientationRequirement);
    NavigateToTask(XYPoint xy, 
                   double endpointOrientation, 
                   bool endpointOrientationRequirement, TravelDirection travelDirection);

    virtual void notStarted(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            RobotState& nextRobotState) override;

    virtual void inProgress(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            RobotState& nextRobotState) override;

    virtual void suspended(std::shared_ptr<Map> map, 
                           std::shared_ptr<Navigator> navigator, 
                           RobotState& nextRobotState, 
                           TaskType& nextTaskType) override;

    virtual void complete(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          RobotState& nextRobotState, 
                          TaskType& nextTaskType) override;

    inline double getEndpointDesiredOrientation() { return endpointDesiredOrientation; }
    inline bool getIsEndpointOrientationRequired() { return isEndpointOrientationRequired; }
    void setEndpoint(double destx, double desty, double destOrientation) {
        endpoint.setX(desty);
        endpoint.setY(desty);
    }

    void printTaskInfo(std::string taskStateName);

private:
    // map data (this data gets stored in robot map)
    XYPoint endpoint; 
    double endpointDesiredOrientation; // angle

    // task behavioral state variables (affects how task executes decisions)
    bool isRobotAtEndpoint;
    bool isEndpointOrientationRequired;

    // helper functions
    //void EndpointPoseCorrection(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) ;
  
    // suspended state reasons
    TaskType newTaskRequest;

    double destinationOrientationTolerance;

    TravelDirection travelDirection;
};

#endif