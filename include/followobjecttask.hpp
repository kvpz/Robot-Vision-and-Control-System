#ifndef FOLLOWOBJECTTASK_HPP
#define FOLLOWOBJECTTASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "enums/objects.hpp"
//#include "enums/robotPoseToWaypoint.hpp"
//#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"
#include "settings.hpp"

class FollowObjectTask : public Task
{
public:
    FollowObjectTask();

    virtual void notStarted(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            RobotState& nextRobotState) override;

    virtual void inProgress(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            RobotState& nextRobotState) override;

    virtual void suspended(std::shared_ptr<Map> map, 
                           std::shared_ptr<Navigator> navigator, 
                           RobotState& nextRobotState, TaskType& nextTaskType) override;

    virtual void complete(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          RobotState& nextRobotState, TaskType& nextTaskType) override;

private:
    // target object type
    ObjectType objectType;

    // left and right x of the bounding frame around the object
    unsigned int x1, x2;

    // stop distance before object
    double stopDistance;
};

#endif