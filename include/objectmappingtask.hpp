#ifndef OBJECTMAPPINGTASK_HPP
#define OBJECTMAPPINGTASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "enums/objects.hpp"
//#include "enums/robotPoseToWaypoint.hpp"
//#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"
#include "settings.hpp"
#include <jsoncpp/json/json.h>
#include <mqueue.h>
#include <sstream>

class ObjectMappingTask : public Task
{
public:
    ObjectMappingTask();
    //ObjectMappingTask(ObjectType);

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

    //bool find_objects(const std::vector<Object>& objects);

    void setObjectGlobalPosition(std::shared_ptr<Map> map, 
                                    ObjectType objectType,
                                    double distanceToObject);

    virtual void printTaskInfo() override;

private:
    // target object type
    ObjectType objectType;

};

#endif