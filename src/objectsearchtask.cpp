#include "objectsearchtask.hpp"
#include <cmath>
ObjectSearchTask::ObjectSearchTask()//ObjectType objectType)
    : object_mq_name(OBJECTSEARCH_MESSAGE_QUEUE), 
    object_mq(mq_open(OBJECTSEARCH_MESSAGE_QUEUE, O_CREAT | O_RDWR | O_NONBLOCK, 0666, nullptr)),
    Task(TaskType::OBJECTSEARCH, OBJECTSEARCHTASK_PRIORITY)
{
    
}

void ObjectSearchTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState)
{
    status = TaskStatus::INPROGRESS;
}

void ObjectSearchTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState)
{
    //std::cout << "(ObjectSearchTask::InProgress) starting" << std::endl;

    // get data from message queue
    Json::Value mqdata = getObjectMQData();
    // collect data for objects that are centered with the robot 
    if(mqdata != Json::Value::null) {
        for (Json::Value& object : mqdata) {
            XYPoint xypoint1;
            XYPoint xypoint2;
            ObjectType objectType(object["object"].asString());

            //objectType = *(new ObjectType(object["object"].asString()));
            xypoint1.setX(object["x1"].asInt());
            xypoint1.setY(object["y1"].asInt());
            xypoint2.setX(object["x2"].asInt());
            xypoint2.setY(object["y2"].asInt());

            // add the object to the map only if the robot is facing the object
            // check if the center of the object is near the center of the frame
            std::cout << "bounding box center xy: " << xypoint1.getX() << " + " << "abs("
                    << xypoint2.getX() << " - " << xypoint1.getX() << ") / 2" << std::endl;
            int boundingBoxCenterXY = xypoint1.getX() + ((xypoint2.getX() - xypoint1.getX()) / 2);
            std::cout << "bounding box center: " << boundingBoxCenterXY << std::endl;
            if(boundingBoxCenterXY < 440 && boundingBoxCenterXY > 400)
                setObjectGlobalPosition(map, objectType, object["distance"].asDouble());
        }
    }

    //std::cout << "(ObjectSearchTask::InProgress) ending" << std::endl;
    /*
    if (!founded) {
        founded = find_objects(objects);
    } 
    */
    // calculate the global position of object(s) detected
    // iterate through all calculated positions 
    // add object(s) to map if there isn't a xy key with approximately the same value

}

void ObjectSearchTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

void ObjectSearchTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState, TaskType& nextTaskType)
{
    
}

Json::Value ObjectSearchTask::getObjectMQData()
{
    std::cout << "(ObjectSearchTask::getObjectMQData) starting" << std::endl;
    char message[40000];
    unsigned int priority;
    ssize_t bytes_received = mq_receive(object_mq, message, 40000, &priority);

    std::cout << "(ObjectSearchTask::getObjectMQData) after mq_receive" << std::endl;


    if (bytes_received == -1) {
        std::cerr << "mq_receive() failed: " << std::strerror(errno) << std::endl;
        return Json::Value::null;
    }
    else if (bytes_received > 0) {
        std::cout << "(ObjectSearchTask::getObjectMQData) data received" << std::endl;
        message[bytes_received] = '\0';
        std::string decoded_message = std::string(message);
        // Convert the string to a list of dictionaries
        Json::Value root;
        Json::CharReaderBuilder builder;
        const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        std::string errors;
        bool success = reader->parse(decoded_message.c_str(), decoded_message.c_str() + decoded_message.size(), &root, &errors);
        if (!success) {
            std::cerr << "Failed to parse JSON message: " << errors << std::endl;
            return Json::Value::null;
        }
        else {
            return root;
        }
    }
    else { 
        return Json::Value::null;
    }
}

/*
bool ObjectSearchTask::find_objects(const std::vector<Object>& objects) {
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

void ObjectSearchTask::setObjectGlobalPosition(std::shared_ptr<Map> map, 
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

void ObjectSearchTask::printTaskInfo()
{
    if(DEBUG_NAVIGATETOTASK) {
        //Task::printTaskInfo();
        //std::cout << "\n====== " << taskStateName << " =======\n" << std::endl;
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}
