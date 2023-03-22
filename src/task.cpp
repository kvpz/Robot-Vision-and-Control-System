#include "task.hpp"

//Task::Task() : status(NOTSTARTED) {}

Task::Task(TaskType ttype)
    : status(NOTSTARTED), taskType(ttype)
{
}

void Task::notStarted(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState){}
void Task::inProgress(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState){}
void Task::suspended(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType){}
void Task::complete(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState, TaskType& nextTaskType){}

// setters
void Task::setStatus(Status s)
{
    status = s;
}

void Task::setEndpoint(double destx, double desty, double destOrientation)
{
    destination.setX(desty);
    destination.setY(desty);
}

void Task::setEndpoint(std::unique_ptr<Map> map)//double x, double y, double orientation)
{
    destination.setX(map->getNextDestinationXY().getX());
    destination.setY(map->getNextDestinationXY().getY());
    //endpointDesiredOrientation = orientation;

    if(DEBUG_TASK) {
        std::cout << "======= Task::setEndpoint =========\n";
        std::cout << "set X: " << map->getNextDestinationXY().getX() << "\n";
        std::cout << "set Y: " << map->getNextDestinationXY().getY() << "\n";
        //std::cout << "setEndpoint: " << endpointDesiredOrientation << "\n";
        std::cout << "===================================\n" << std::endl;
    }
}

// getters
Status Task::getStatus() const { return status; }
Waypoint Task::getDestination() const { return destination; } 
TaskType Task::getTaskType() const { return taskType; }
std::string Task::getName() { return taskTypeToString(taskType); }

void Task::printTaskInfo(Task& task)
{
    // print status of this type of task
    switch(task.getTaskType()) {
        case NAVIGATETO:
            std::cout << "\n====== Travel Task Updater ======\n";
            break;
        case PATHCORRECTION:
            std::cout << "\n====== PathCorrection Task Updater ======\n";
            break;  
        case DROPPAYLOAD:
            std::cout << "\n====== Drop Payload Task Updater ======\n";
            break;
        case GRASP:
            std::cout << "\n====== Grasp Task Updater ======\n";
            break;
        case STACKPED: 
            std::cout << "\n====== Stack Pedestal Task Updater ======\n";
            break;
    }

    std::cout << "Task destination (X,Y): (" << task.getDestination().getX() 
            << ", " << task.getDestination().getY() << ")\n";
    std::cout << "Task status: " << statusToString(task.getStatus()) << "\n";
    std::cout << "Task name: " << task.getName() << "\n";
    std::cout << "=================================\n";
    std::cout << std::endl;
}