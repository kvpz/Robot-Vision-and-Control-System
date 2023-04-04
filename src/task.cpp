#include "task.hpp"

Task::Task() 
 : status(TaskStatus::NOTSTARTED),
 taskType(NA), priority_(UINT_MAX)
 {}

Task::Task(TaskType ttype, unsigned int priority)
    : status(TaskStatus::NOTSTARTED), taskType(ttype),
    priority_(priority)
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
            std::cout << "\n====== Travel Task Updater ======\n";
            break;
        case PATHCORRECTION:
            std::cout << "\n====== PathCorrection Task Updater ======\n";
            break;  
        case DROPCHIP:
            std::cout << "\n====== Drop Chip Task Updater ======\n";
            break;
        case GRASP:
            std::cout << "\n====== Grasp Task Updater ======\n";
            break;
        case STACKPED: 
            std::cout << "\n====== Stack Pedestal Task Updater ======\n";
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