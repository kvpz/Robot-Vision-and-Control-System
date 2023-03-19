#include "task.hpp"

Task::Task() {}

Task::Task(TaskType ttype)
    : status(NOTSTARTED), taskType(ttype)
{
}

void Task::notStarted(Map* map, Navigator* navigator, RobotState& robotState){}
void Task::inProgress(Map* map, Navigator* navigator, RobotState& robotState){}
void Task::suspended(){}
void Task::complete(){}

// setters
void Task::setStatus(Status s)
{
    status = s;
}

void Task::setEndpoint(double x, double y, double orientation)
{
    destination.setX(x);
    destination.setY(y);
    endpointOrientation = orientation;
}

// getters
Status Task::getStatus() const { return status; }
Waypoint Task::getDestination() const { return destination; } 
TaskType Task::getTaskType() const { return taskType; }
inline std::string Task::getName() { return taskTypeToString(taskType); }
inline double Task::getEndpointOrientation() { return endpointOrientation; }

void Task::printTaskInfo(Task& task)
{
    // print status of this type of task
    switch(task.getTaskType()) {
        case TRAVEL:
            std::cout << "\n====== Travel Task Updater ======\n";
            break;
        case CORRECTPATH:
            std::cout << "\n====== CorrectPath Task Updater ======\n";
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