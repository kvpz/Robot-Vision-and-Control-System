#include "objectmappingtask.hpp"
#include <cmath>
ObjectMappingTask::ObjectMappingTask()
    : Task(TaskType::OBJECTMAPPING, OBJECTMAPPINGTASK_PRIORITY)
{
    
}

void ObjectMappingTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState)
{
    status = TaskStatus::INPROGRESS;
}

void ObjectMappingTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState)
{
    std::multiset<BoundingBox> boundingBoxes = visionData->getObjectsDetected();

    for(BoundingBox bbox : boundingBoxes) {
        XYPoint<int> boundingBoxCenterXY = bbox.getCenterXY();

        // map object if it is nearly centered with the robot
        if(boundingBoxCenterXY.getX() < 440 && boundingBoxCenterXY.getX() > 400)
            setObjectGlobalPosition(map, bbox.getObjectType(), bbox.getDistanceFromCamera());
    }
}

void ObjectMappingTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void ObjectMappingTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}


/*
bool ObjectMappingTask::find_objects(const std::vector<Object>& objects) {
    std::vector<Object> ducks;
    std::vector<ObjectType> ducks;


    for (const auto& object_viewed : objects) {
        if (object_viewed.object == "duck") {
            ducks.push_back(object_viewed);
        }
    }

    if (!ducks.empty()) {
        const Object& duck = *std::min_element(ducks.begin(), ducks.end(),
            [](const Object& x, const Object& y) { return x.distance < y.distance; });
        const int x1 = duck.x1;
        const int x2 = duck.x2;
        const int distance = duck.distance;
        const int center = x1 + (x2 - x1) / 2;
        std::cout << "distance: " << distance << std::endl;
        if (center < 300) {
            send_command("Z");
        } else if (center > 550) {
            send_command("C");
        } else if (center < 380) {
            send_command("R");
        } else if (center > 468) {
            send_command("L");
        } else if (distance > 20) {
            send_command("F");
            if (distance > 40) {
                send_command("c");
            } else {
                send_command("a");
            }
        } else {
            std::cout << "founded" << std::endl;
            send_command("S");
            return true;
        }
    } else {
        send_command("S");
    }
    return  false;
}
*/

void ObjectMappingTask::setObjectGlobalPosition(std::shared_ptr<Map> map, 
                                                  ObjectType objectType,
                                                  double distanceToObject)
{
    XYPoint xypoint;
    xypoint.setX(map->RobotX() + distanceToObject * cos(map->getRobotOrientation() / 180.0 * M_PI));
    xypoint.setY(map->RobotY() + distanceToObject * sin(map->getRobotOrientation() / 180.0 * M_PI));
    map->addObjectDetected(objectType, xypoint);

    std::cout << " ======== map content ========= " << std::endl;
    map->printObjectMap();
    std::cout << "-----" << std::endl;
    map->printOccupancyGrid();
    std::cout << "======== end map content =========" << std::endl;
}

void ObjectMappingTask::printTaskInfo()
{
    if(DEBUG_OBJECTMAPPINGTASK) {
        Task::printTaskInfo(*this);
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}
