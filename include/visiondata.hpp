#ifndef VISIONDATA_HPP
#define VISIONDATA_HPP
#include <iostream>
#include <optional>
#include <map>
#include <set>
#include <unordered_map>
#include <functional>
#include "xypoint.hpp"
#include "enums/objects.hpp"
#include "enums/attractionColors.hpp"
#include "utility.hpp"
#include <mqueue.h>
#include <jsoncpp/json/json.h>
#include <chrono>
#include <ctime>
#include "boundingbox.hpp"
#include "settings.hpp"
#include <memory>

/*
    This class is used to read vision data from vision components
    of the system. In the current state of the AMIRA system, the 
    vision data is expected to come from a message queue. There are 
    two types of message queues:
        1. object detection queue
        2. attraction detection queue
*/
class VisionData
{
public:
    VisionData();

    std::multiset<BoundingBox> getObjectsDetected();

    AttractionColors getAttractionColorMQData();

private:
    
    Json::Value getObjectMQData();

    // set of bounding boxes
    std::multiset<BoundingBox> boundingBoxes;

    // time data was last collected (or assume all data is the latest)
    // 

    // vision message queues
    // object detection message queue
    mqd_t object_detect_mq;
    const char* object_detect_mq_name;

    // attraction color detection message queue
    mqd_t attraction_color_mq;
    const char* attraction_color_mq_name;

    // last time data was read from the message queue
    std::chrono::system_clock::time_point last_read_from_object_mq;
};

#endif