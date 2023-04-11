#ifndef OBJECTSEARCHTASK_HPP
#define OBJECTSEARCHTASK_HPP

#include "robot.hpp"
#include "task.hpp"
#include "enums/objects.hpp"
//#include "enums/robotPoseToWaypoint.hpp"
//#include "enums/robotState.hpp"
#include "navigator.hpp"
#include "map.hpp"
#include "settings.hpp"
#include <json/json.h>
#include <mqueue.h>
#include <sstream>

struct XYPoint;

class ObjectSearchTask : public Task
{
public:
    ObjectSearchTask();
    //ObjectSearchTask(ObjectType);

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

    //bool find_objects(const std::vector<Object>& objects);

    Json::Value getObjectMQData();

    void setObjectGlobalPosition(std::shared_ptr<Map> map, 
                                    ObjectType objectType,
                                    double distanceToObject);

    virtual void printTaskInfo() override;

private:
    // target object type
    ObjectType objectType;

    // message queue and name
    mqd_t object_mq;
    const char* object_mq_name;

};

#endif