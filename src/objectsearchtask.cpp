#include "objectsearchtask.hpp"

ObjectSearchTask::ObjectSearchTask(ObjectType objectType)
    : object_mq_name(messageQueueName), 
    object_mq(mq_open(messageQueueName, O_CREAT | O_RDWR | O_NONBLOCK, 0666, nullptr)),
    Task(TaskType::OBJECTSEARCH, OBJECTSEARCHTASK_PRIORITY)
{
    
}

void ObjectSearchTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState)
{
    
}

void ObjectSearchTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState)
{
    // get data from message queue
    Json::Value mqdata = getObjectMQData();
    // collect data for objects that are centered with the robot 
    std::vector<Object> objects;
    for (Json::Value& object : mqdata) {
        XYPoint xypoint;

        Object obj;
        obj.object = object["object"].asString();
        xypoint.setX(object["x1"].asInt());
        xypoint.setY(object["x2"].asInt());
        xypoint.setX(object["x1"].asInt());
        xypoint.setY(object["x2"].asInt());
        obj.distance = object["distance"].asInt();
        objects.push_back(obj);
    }

    if (!founded) {
        founded = find_objects(objects);
    } 
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
    char message[40000];
    unsigned int priority;
    ssize_t bytes_received = mq_receive(mq, message, 40000, &priority);
    if (bytes_received == -1) {
        std::cerr << "mq_receive() failed: " << std::strerror(errno) << std::endl;
    }
    if (bytes_received > 0) {
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
            return null;
        }
        else {
            return root
        }
    }
}

bool ObjectSearchTask::find_objects(const std::vector<Object>& objects) {
    std::vector<Object> ducks;
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

XYPoint ObjectSearchTask::getObjectGlobalPosition(std::shared_ptr<Map> map, 
                                                  ObjectType objectType,
                                                  double distanceToObject)
{
    XYPoint xypoint;
    xypoint.setX(map->RobotX() + distanceToObject * cos(map->getRobotOrientation()));
    xypoint.setY(map->RobotY() + distanceToObject * sin(map->getRobotOrientation()));
    map->addObjectDetected(objectType, xypoint);
}