#ifndef ATTRACTIONCOLORTASK
#define ATTRACTIONCOLORTASK
#include <iostream>
#include <mqueue.h>
#include "robot.hpp"
#include "task.hpp"
#include "enums/robotPoseToWaypoint.hpp"
#include "enums/robotState.hpp"
#include "enums/tasktype.hpp"
#include "map.hpp"
#include "settings.hpp"

/*
    This runs only under the following conditions:
    (1) robot does not know color of the attraction(s)
    (2) robot is facing the direction of one of the attractions
    (3) robot is near one of the attractions
    (4) there are not objects overlapping the view of the attraction areas

    This task can be added to the task queue by the task scheduler 
*/
class AttractionColorTask : public Task
{
public:
    AttractionColorTask(const char* messageQueueName);

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
                           RobotState& nextRobotState, 
                           TaskType& nextTaskType) override;

    virtual void complete(std::shared_ptr<Map> map, 
                          std::shared_ptr<Navigator> navigator, 
                          std::shared_ptr<VisionData> visionData,
                          RobotState& nextRobotState, 
                          TaskType& nextTaskType) override;

    virtual void printTaskInfo() override;

private:


};

#endif