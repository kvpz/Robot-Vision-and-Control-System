#include "dropchiptask.hpp"

DropChipTask::DropChipTask() : Task(DROPCHIP) {}

DropChipTask::DropChipTask(XYPoint xy, 
                           double endpointOrientation, 
                           bool endpointOrientationRequirement)
    : Task(DROPCHIP)
{
    payloadLocation = xy;
    this->endpointOrientation = endpointOrientation;
    this->endpointOrientationRequirement = endpointOrientationRequirement;
    isDeployingPayload = false;
}

void DropChipTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    // get information about current location from the map
    // if robot is at bottom left of playing field, then it should request information 
    // about the bottom left of the map (i.e. map->getBottomLeftFieldColor())
    // else about the top left of the map (i.e. map->getTopLeftFieldColor())
    if(map->RobotY() < 50.0){
        attractionColor = map->getBottomLeftAttractionColor();
    }
    else {
        attractionColor = map->getTopLeftAttractionColor();
    }
    attractionColor = AttractionColors::RED;

    // a color detection script will be running at the same time as any other task;
    // the color detection script is what will report the color of the attractions in the map

    status = TaskStatus::INPROGRESS;
}

void DropChipTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    if(isDeployingPayload == false) {
        if(redDeploymentAttempts == 0 && attractionColor == AttractionColors::RED) { 
            nextRobotState = OPENING_LEFT_RECEPTACLE; // dispense left chips
            ++redDeploymentAttempts;
        }
        else if (greenDeploymentAttempts == 0 && attractionColor == AttractionColors::GREEN) {
            attractionColor = AttractionColors::GREEN;
            nextRobotState = OPENING_RIGHT_RECEPTACLE; // dispense right chips
            ++greenDeploymentAttempts;
        }
        else if(redDeploymentAttempts < greenDeploymentAttempts) {
            attractionColor = AttractionColors::RED;
            nextRobotState = OPENING_LEFT_RECEPTACLE;
            ++redDeploymentAttempts;
        }
        else if(greenDeploymentAttempts < redDeploymentAttempts) {
            attractionColor = AttractionColors::GREEN;
            nextRobotState = OPENING_RIGHT_RECEPTACLE;
            ++greenDeploymentAttempts;
        }

        isDeployingPayload = true;
    }
    else if(runtime < 2000) {
        // do nothing for fixed amount of time after which we
        // can presume the chips have been dropped 
    }
    else {
        if(attractionColor == AttractionColors::RED) { 
            nextRobotState = CLOSING_LEFT_RECEPTACLE; 
            status = TaskStatus::COMPLETE;
        }
        else if (attractionColor == AttractionColors::GREEN) {
            nextRobotState = CLOSING_RIGHT_RECEPTACLE; 
            status = TaskStatus::COMPLETE;
        }
    }

    runtime += 10;
}

void DropChipTask::suspended(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState, TaskType& nextTaskType) 
{
    // The robot can enter a suspended state if it is not near the deployment endpoint
    // if the robot is not near the point where it should drop the chips, a navigateTo
    // task should be requested
}

void DropChipTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        RobotState& nextRobotState, TaskType& nextTaskType) 
{
    
}