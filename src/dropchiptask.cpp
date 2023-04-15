#include "dropchiptask.hpp"

DropChipTask::DropChipTask() : Task(DROPCHIP, DROPCHIPTASK_PRIORITY) {}

DropChipTask::DropChipTask(XYPoint<double> xy, 
                           double endpointOrientation, 
                           bool endpointOrientationRequirement)
    : Task(TaskType::DROPCHIP, DROPCHIPTASK_PRIORITY)
{
    payloadLocation = xy;
    this->endpointOrientation = endpointOrientation;
    this->endpointOrientationRequirement = endpointOrientationRequirement;
    isDeployingPayload = false;

}

void DropChipTask::notStarted(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState) 
{
    // get information about nearest attraction from the map
    // if robot is at bottom left of playing field, then it should request information 
    // about the bottom left of the map (i.e. map->getBottomLeftFieldColor())
    // else about the top left of the map (i.e. map->getTopLeftFieldColor())
    // Note: a color detection script will be running at the same time as any other task;
    // the color detection script is what will report the color of the attractions in the map
    if(map->RobotY() < 50.0){
        attractionColor = map->getBottomLeftAttractionColor();
    }
    else {
        attractionColor = map->getTopLeftAttractionColor();
    }

    // store the point of the attraction location in the map
    map->setDestinationXY(payloadLocation);

    status = TaskStatus::INPROGRESS;
}

void DropChipTask::inProgress(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState) 
{
    // if robot not at the attraction xypoint, create NavigateTo task
    /*if(!navigator->isRobotNearPoint(map)) {
        nextRobotState = RobotState::STOP;
        status = TaskStatus::SUSPENDED;
        newTaskRequest = TaskType::NAVIGATETO;
    }
    */
    //else 
    if(isDeployingPayload == false) {
        if(redDeploymentAttempts == 0 && attractionColor == AttractionColors::RED) { 
            nextRobotState = OPENING_LEFT_RECEPTACLE; // dispense left chips
            ++redDeploymentAttempts;
        }
        else if (greenDeploymentAttempts == 0 && attractionColor == AttractionColors::GREEN) {
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
            //newTaskRequest = TaskType::NAVIGATETO;
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
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType) 
{
    // The robot can enter a suspended state if it is not near the deployment endpoint
    // if the robot is not near the point where it should drop the chips, a navigateTo
    // task should be requested
    //nextTaskType = newTaskRequest;
    nextRobotState = STOP;
}

void DropChipTask::complete(std::shared_ptr<Map> map, 
                        std::shared_ptr<Navigator> navigator, 
                        std::shared_ptr<VisionData> visionData,
                        RobotState& nextRobotState, TaskType& nextTaskType) 
{
    
}

void DropChipTask::printTaskInfo()
{
    if(DEBUG_DROPCHIPTOTASK) {
        Task::printTaskInfo(*this);
        std::cout << "status: " << statusToString(this->getStatus()) << "\n";
        std::cout << "payload location: " << payloadLocation << "\n";
        std::cout << "is robot deploying chips: " << isDeployingPayload << "\n";
        std::cout << "\n==========================================\n" << std::endl;
    }
}
