#include "visiondata.hpp"
#include <string>

VisionData::VisionData()
: object_detect_mq_name(OBJECTMAPPING_MESSAGE_QUEUE), 
  object_detect_mq(mq_open(OBJECTMAPPING_MESSAGE_QUEUE, O_CREAT | O_RDWR | O_NONBLOCK, 0666, nullptr)),
  attraction_color_mq_name(ATTRACTIONCOLOR_MESSAGE_QUEUE), 
  attraction_color_mq(mq_open(ATTRACTIONCOLOR_MESSAGE_QUEUE, O_CREAT | O_RDWR | O_NONBLOCK, 0666, nullptr)),
  last_read_from_object_mq(std::chrono::system_clock::now()),
  last_read_from_attraction_mq(std::chrono::system_clock::now())
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
std::multiset<BoundingBox> VisionData::getObjectsDetected()
{    
    // try reading from the message queue
    Json::Value objectMQData = getObjectMQData();

    // collect data for objects that are centered with the robot 
    if(objectMQData != Json::Value::null) {
        objectsDetected.clear();
        
        for (Json::Value& object : objectMQData) {

            XYPoint<int> xypoint1;
            XYPoint<int> xypoint2;
            ObjectType objectType(object["object"].asString()); 
            xypoint1.setX(std::stoi(object["x1"].asString()));
            xypoint1.setY(std::stoi(object["y1"].asString()));
            xypoint2.setX(std::stoi(object["x2"].asString()));
            xypoint2.setX(std::stoi(object["y2"].asString()));
            BoundingBox boundingBox(xypoint1, xypoint2, objectType, object["distance"].asDouble());
            objectsDetected.insert(boundingBox);
            // add the object to the map only if the robot is facing the object
            // check if the center of the object is near the center of the frame

            // map object if it is nearly centered with the robot
            //if(boundingBoxCenterXY < 440 && boundingBoxCenterXY > 400)
            //    setObjectGlobalPosition(map, objectType, object["distance"].asDouble());
        }
    }
    else {
        // return what was last read from the message queue
    }

    // return an empty result if last read from message queue happened long ago
    if(std::chrono::system_clock::now() > last_read_from_object_mq + std::chrono::seconds{3}){
        //objectsDetected = std::multiset<BoundingBox>();
    }

    printTaskInfo();
    return objectsDetected;
}

AttractionColors VisionData::getAttractionColorDetected()
{
    // try reading from the message queue
    char mqdata = getAttractionColorMQData();
    
    if(mqdata != '\0') {
        // collect data for objects that are centered with the robot 
        if(mqdata == 'R') {
            attractionColorDetected = AttractionColors::RED;
        }
        else if(mqdata == 'G') {
            attractionColorDetected = AttractionColors::GREEN;
        }
        else {
            attractionColorDetected = AttractionColors::NONE;
        }
    }
    else {
        // return the last message that was read from the message queue
    }

    // return an empty result if last read from message queue happened long ago
    if(std::chrono::system_clock::now() > last_read_from_attraction_mq + std::chrono::seconds{3}){
        attractionColorDetected = AttractionColors::NONE;
    }

    return attractionColorDetected;
}

Json::Value VisionData::getObjectMQData()
{
    const int mq_max_size = 40000;
    const int mq_msg_size = 40000;
    char message[mq_msg_size];
    //memset(message, 0, mq_msg_size);
    unsigned int priority;
    ssize_t bytes_received = mq_receive(object_detect_mq, message, mq_max_size, &priority);

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
            last_read_from_object_mq = std::chrono::system_clock::now();
            return root;
        }
    }
    else { 
        return Json::Value::null;
    }
}

char VisionData::getAttractionColorMQData()
{
    char result = '\0';
    const int mq_max_size = 10000;
    const int mq_msg_size = 102400;

    char buffer[mq_msg_size];
    memset(buffer, 0, mq_msg_size);
    ssize_t bytes_received = mq_receive(attraction_color_mq, buffer, mq_max_size, nullptr);
    if (bytes_received == -1) {
        std::cout << "Error receiving message from queue: " << strerror(errno) << std::endl;
    }
    else {
        if(buffer[0] == 'R') {
            result = 'R';
        }
        else if(buffer[0] == 'G') {
            result = 'G';
        }

        last_read_from_attraction_mq = std::chrono::system_clock::now();
    }
    
    return result;
}

void VisionData::printTaskInfo()
{
    if(DEBUG_VISIONDATA) {
        std::cout << "========= Vision Data Service Info ========\n";
        std::cout << "objects detected: ";
        std::cout << "objectsDetected count: " << objectsDetected.size() << std::endl;
        for(BoundingBox object : objectsDetected) {
            std::cout << "object type: " << object.getObjectType().toString() << "\n"; 
            std::cout << "distance from object: " << object.getDistanceFromCamera() << "\n";
            std::cout << "xy1-x: " << object.getXY1().getX() << "\n";
        }
        std::cout << "\n==========================================\n" << std::endl;
    }
}