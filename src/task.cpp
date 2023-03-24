#include "task.hpp"

Task::Task(TaskType ttype)
    : status(TaskStatus::NOTSTARTED), taskType(ttype)
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


    std::cout << "Task status: " << statusToString(task.getStatus()) << "\n";
    std::cout << "Task name: " << task.getName() << "\n";
    std::cout << "=================================\n";
    std::cout << std::endl;
}