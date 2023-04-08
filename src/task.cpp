#include "task.hpp"

Task::Task() 
 : status(TaskStatus::NOTSTARTED),
 taskType(NA), priority_(UINT_MAX), readyToBeDeleted(false)
 {}

Task::Task(TaskType ttype, unsigned int priority)
    : status(TaskStatus::NOTSTARTED), taskType(ttype),
    priority_(priority), readyToBeDeleted(false)
{
}

void Task::notStarted(std::shared_ptr<Map> map, 
                      std::shared_ptr<Navigator> navigator, RobotState& nextRobotState){}
void Task::inProgress(std::shared_ptr<Map> map, 
                      std::shared_ptr<Navigator> navigator, RobotState& nextRobotState){}
void Task::suspended(std::shared_ptr<Map> map, 
                     std::shared_ptr<Navigator> navigator, 
                     RobotState& nextRobotState, TaskType& nextTaskType){}
void Task::complete(std::shared_ptr<Map> map, 
                    std::shared_ptr<Navigator> navigator, 
                    RobotState& nextRobotState, TaskType& nextTaskType){}

// setters
void Task::setStatus(TaskStatus s)
{
    status = s;
}

// getters
TaskStatus Task::getStatus() const { return status; }
TaskType Task::getTaskType() const { return taskType; }
std::string Task::getName() { return taskTypeToString(taskType); }

void Task::printTaskInfo(Task& task)
{
    // print status of this type of task
    switch(task.getTaskType()) {
        case NAVIGATETO:
            std::cout << "\n====== NavigateTo Task ======\n";
            break;
        case PATHCORRECTION:
            std::cout << "\n====== PathCorrection Task ======\n";
            break;  
        case POSECORRECTION:
            std::cout << "\n====== PoseCorrection Task ======\n";
            break;
        case ATTRACTIONCOLOR:
            std::cout << "\n====== AttractionColor Task ======\n";
            break;
        case DROPCHIP:
            std::cout << "\n====== Drop Chip Task ======\n";
            break;
        case GRASP:
            std::cout << "\n====== Grasp Task ======\n";
            break;
        case STACKPED: 
            std::cout << "\n====== Stack Pedestal Task ======\n";
            break;
        case CONTROLMANDIBLES:
            std::cout << "\n====== Control Mandibles Task ======\n";
            break;
    }


    std::cout << "Task status: " << statusToString(task.getStatus()) << "\n";
    std::cout << "Task name: " << task.getName() << "\n";
    std::cout << "=================================\n";
    std::cout << std::endl;
}

void Task::printTaskInfo()
{
    printTaskInfo(*this);
}