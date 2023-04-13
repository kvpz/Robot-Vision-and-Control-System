#include "followobjecttask.hpp"

FollowObjectTask::FollowObjectTask(ObjectType objectToFollow)
    : Task(TaskType::FOLLOWOBJECT, FOLLOWOBJECTTASK_PRIORITY)
{
    timeCounter = 0;
    objectType = objectToFollow;
}

void FollowObjectTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState)
{
    status = TaskStatus::INPROGRESS;
}

void FollowObjectTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState)
{
    // check if we found the desired object
    bool isDesiredObjectDetected = false;
    for(BoundingBox bbox : visionData->getObjectsDetected()) {
        if(bbox.getObjectType().getObjectType() == objectType.getObjectType()) {
            isDesiredObjectDetected = true;
            break;
        }
    }

    if(!isDesiredObjectDetected) {
        
        timeCounter += 10;
        // rotate robot until an object is detected (CW or CCW?)
        nextRobotState = RobotState::ROTATE_CW;
    }
    else {
        timeCounter = 0;
        
        BoundingBox closestDesiredObject({0,0}, {0,0}, OBJECTTYPES::NA, 100000);
        // iterate through all the objects that were detected
        int i = 0;
        for(BoundingBox object : visionData->getObjectsDetected()) {
            //std::cout << "iterating through boundingboxobjects " << ++i << std::endl;
            //std::cout << "object distance: " << object.getDistanceFromCamera() << "vs" << closestDesiredObject.getDistanceFromCamera() << std::endl;
            //std::cout << "object types: " << object.getObjectType() << std::endl;
            // if desired object is detected, choose the closest one
            if(object.getObjectType() == objectType) {
                if(object.getDistanceFromCamera() < closestDesiredObject.getDistanceFromCamera()) {
                    closestDesiredObject = object;
                    std::cout << "closest desired object detected" << std::endl;
                }
            }
        }
        std::cout << "(followobjecttask) objectType: " << closestDesiredObject.getObjectType().toString() << std::endl;
        distanceToClosestDesiredObject = closestDesiredObject.getDistanceFromCamera();
            
        x1 = closestDesiredObject.getXY1().getX();
        x2 = closestDesiredObject.getXY2().getX();
        std::cout << "(followobjecttask) x1: " << x1 << std::endl;
        std::cout << "(followobjecttask) x2: " << x2 << std::endl;
        if(closestDesiredObject.getCenterXY().getX() < 300) {
            nextRobotState = RobotState::ROTATE_CCW;
        }
        else if(closestDesiredObject.getCenterXY().getX() > 550) {
            nextRobotState = RobotState::ROTATE_CW;
        }            
        else if(closestDesiredObject.getCenterXY().getX() < 380) {
            nextRobotState = RobotState::MOVE_RIGHT;
        }
        else if(closestDesiredObject.getCenterXY().getX() > 468) {
            nextRobotState = RobotState::MOVE_LEFT;
        }
        else if(closestDesiredObject.getDistanceFromCamera() > 20){
            nextRobotState = RobotState::MOVE_FORWARD;
            // speed adjustments below...
        }
        else {
            nextRobotState = RobotState::STOP;
            status = TaskStatus::COMPLETE;
        }
    }
}

void FollowObjectTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void FollowObjectTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void FollowObjectTask::printTaskInfo()
{
    if(DEBUG_FOLLOWOBJECTTASK) {
        Task::printTaskInfo(*this);
        std::cout << "desired object type: " << objectType.toString() << "\n";
        std::cout << "x1: " << x1 << " x2: " << x2 << "\n";
        std::cout << "distance to desired object: " << distanceToClosestDesiredObject << "\n";
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}
