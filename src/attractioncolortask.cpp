#include "attractioncolortask.hpp"

AttractionColorTask::AttractionColorTask(const char* messageQueueName)
    : //attraction_color_mq_name(messageQueueName), 
    //attraction_color_mq(mq_open(messageQueueName, O_CREAT | O_RDWR | O_NONBLOCK, 0666, nullptr)),
    Task(TaskType::ATTRACTIONCOLOR, ATTRACTIONCOLORTASK_PRIORITY)
{
    // this function initializes the message queue for reading

}

void AttractionColorTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData, 
                        RobotState& nextRobotState)
{
    // start receiving data from the message queue that contains
    // color data from each frame 
    std::cout << "(attractionColorTask::not started) in not started" << std::endl;
    status = TaskStatus::INPROGRESS;
}

void AttractionColorTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState) 
{
    // get data from the message queue
    //AttractionColors detectedColor = getAttractionColorMQData(visionData);
    AttractionColors detectedColor = visionData->getAttractionColorDetected();
    
    // if the last message reports a ratio of 
    // red and green (the color of the attractions) pixels that
    // represents a difference greater than 10% (hardcoded), then
    // the map object needs to be updated so that it contains data
    // about the color of the attraction that the robot is facing. 
    // If the robot is facing the top left quadrant (Q2), then the map
    // should have the value for the bottom/top left attractions
    // assigned to it depending on the robot's current orientation. 
    // The robot should also be at a reasonable distance from the attraction
    // like 120cm or less and not have other red objects in front of it. 
    // Other red objects may skew the average value the robot calculates. 

    // Sample:
    // if robot is facing Q2 && robot near attraction && color GREEN 
    // then record data in map for each attraction

    // if the robot is not near any of the attractions, then do not attempt
    // to interpret and react on the input color data


    switch(quadrant_identifier(map->getRobotOrientation())) {
        case 2:
            if(detectedColor == AttractionColors::GREEN) {
                map->setTopLeftAttractionColor(AttractionColors::GREEN);
                map->setBottomLeftAttractionColor(AttractionColors::RED);
            }
            else if(detectedColor == AttractionColors::RED) {
                map->setTopLeftAttractionColor(AttractionColors::RED);
                map->setBottomLeftAttractionColor(AttractionColors::GREEN);
            }
            break;

        case 3:
            if(detectedColor == AttractionColors::GREEN) {
                map->setBottomLeftAttractionColor(AttractionColors::GREEN);
                map->setTopLeftAttractionColor(AttractionColors::RED);
            }
            else if(detectedColor == AttractionColors::RED) {
                map->setBottomLeftAttractionColor(AttractionColors::RED);
                map->setTopLeftAttractionColor(AttractionColors::GREEN);            
            }
            break;

        default:
            break;
    }

    // if a color was assigned to the attractions
    // change the state of the task
    if(map->isAttractionColorKnown()) {
        std::cout << "AttractionColorTask::inprogress attraction color is known" << std::endl;
        status = TaskStatus::COMPLETE;
    }
}

void AttractionColorTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, 
                        TaskType& nextTaskType) 
{
    // this task could be in a suspended state for the following conditions:
    // (1) robot needs the color of an attraction area
    // (2) robot is not facing the quadrants with an attraction (Q2 & Q3)
    // It would be more efficient to leave this task in suspended because if 
    // the notStarted function contains expensive computations, then the overhead
    // of the initialization code can be avoided.

    
}

void AttractionColorTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, 
                        TaskType& nextTaskType) 
{
    // close the connection to the message queue

    //mq_close(attraction_color_mq);
    //mq_unlink(attraction_color_mq_name);
}


void AttractionColorTask::printTaskInfo()
{
    if(DEBUG_ATTRACTIONCOLORTASK) {
        Task::printTaskInfo(*this);
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}
