#include "visiondata.hpp"

VisionData::VisionData()
: object_mq_name(OBJECTMAPPING_MESSAGE_QUEUE), 
  object_mq(mq_open(OBJECTMAPPING_MESSAGE_QUEUE, O_CREAT | O_RDWR | O_NONBLOCK, 0666, nullptr)),
  last_read_from_mq(std::chrono::system_clock::now())
{

}

/*
    Reads data from the message queue. 
    
    If there is no data in the message queue, then it will return the last data that 
    was received by the object detection message queue IF the data is not stale (old)
    
    if there is no data in the data structure containing past message queue data, then an
    empty multiset will be returned. 

    if the data last collected was from a long time ago, then the function will
    return an empty multiset. 
*/
std::multiset<BoundingBox, BoundingBoxCompare> VisionData::getObjectsDetected()
{
    // try reading from the message queue
    Json::Value objectMQData = getObjectMQData();
    // if there is data, put it in the multiset
    // set the last time read from mq variable to time now
    
    // collect data for objects that are centered with the robot 
    if(objectMQData != Json::Value::null) {
        for (Json::Value& object : objectMQData) {
            XYPoint xypoint1;
            XYPoint xypoint2;
            ObjectType objectType(object["object"].asString());
            xypoint1.setX(object["x1"].asInt());
            xypoint1.setY(object["y1"].asInt());
            xypoint2.setX(object["x2"].asInt());
            xypoint2.setY(object["y2"].asInt());

            BoundingBox boundingBox(xypoint1, xypoint2, objectType);

            // add the object to the map only if the robot is facing the object
            // check if the center of the object is near the center of the frame
            int boundingBoxCenterXY = xypoint1.getX() + ((xypoint2.getX() - xypoint1.getX()) / 2);

            // map object if it is nearly centered with the robot
            if(boundingBoxCenterXY < 440 && boundingBoxCenterXY > 400)
                setObjectGlobalPosition(map, objectType, object["distance"].asDouble());
        }
    }

    if(std::chrono::system_clock::now() > last_read_from_mq + std::chrono::seconds{3}){
        // maybe clear the data structure
        return std::multiset<BoundingBox>();
    }
    else if() {

    }

    return boundingBoxes;
}

Json::Value VisionData::getObjectMQData()
{
    char message[40000];
    unsigned int priority;
    ssize_t bytes_received = mq_receive(object_mq, message, 40000, &priority);

    if (bytes_received == -1) {
        //std::cerr << "mq_receive() failed: " << std::strerror(errno) << std::endl;
        return Json::Value::null;
    }
    else if (bytes_received > 0) {
        //std::cout << "(ObjectMappingTask::getObjectMQData) data received" << std::endl;
        message[bytes_received] = '\0';
        std::string decoded_message = std::string(message);
        // Convert the string to a list of dictionaries
        Json::Value root;
        Json::CharReaderBuilder builder;
        const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        std::string errors;
        bool success = reader->parse(decoded_message.c_str(), decoded_message.c_str() + decoded_message.size(), &root, &errors);
        if (!success) {
            //std::cerr << "Failed to parse JSON message: " << errors << std::endl;
            return Json::Value::null;
        }
        else {
            last_read_from_mq = std::chrono::system_clock::now();
            return root;
        }
    }
    else { 
        return Json::Value::null;
    }
}