#include "taskmanager.hpp"
#include "navigatetotask.hpp"
#include "pathcorrectiontask.hpp"

//TaskManager::TaskManager() 
//{ }

/*
    robot info required to be passed to all notStarted:
    - current x and y position
    - angle to destination
    
    robot info required to be passed to all inProgress:
    - current x and y position
    - 

    This function should avoid implementing task specific behaviors. All actions
    or updates needed to be performed by a task should be done in their respective state functions. 

    Not all tasks may achieve all status. 
*/
void TaskManager::executeCurrentTask(std::unique_ptr<Map> map, std::unique_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    TaskType nextTaskType = NA;
    std::unique_ptr<Task> newTask;
    
    switch(task_queue.top()->getStatus()) {
        case NOTSTARTED:
            task_queue.top()->notStarted(std::move(map), std::move(navigator), nextRobotState);
            // set task to in progress
            break;  
            
        case INPROGRESS:
            task_queue.top()->inProgress(std::move(map), std::move(navigator), nextRobotState);
            break;

        case SUSPENDED:
            std::cout << "\n======= TaskManager::executeCurrentTask ======= path correction\n" << std::endl;
            task_queue.top()->suspended(std::move(map), std::move(navigator), nextRobotState, nextTaskType);

            // Make a new task if required by current task.
            // This is typical when a task enters a suspended state.
            newTask = taskFactory(nextTaskType);
            if(newTask != nullptr) {
                newTask->setEndpoint(task_queue.top()->getDestination().getX(), task_queue.top()->getDestination().getY(), task_queue.top()->getEndpointDesiredOrientation());
                addTask(std::move(newTask));
            }

            break;

        case COMPLETE:
            task_queue.top()->complete(std::move(map), std::move(navigator), nextRobotState, nextTaskType);
            task_queue.pop();

            // Make a new task if required by current task.
            // This is typical when a task enters a suspended state.
            // This should only happen if nextRobotState == COMPLETE
            newTask = taskFactory(nextTaskType);
            if(newTask != nullptr) {
                newTask->setEndpoint(task_queue.top()->getDestination().getX(), task_queue.top()->getDestination().getY(), task_queue.top()->getEndpointDesiredOrientation());
                addTask(std::move(newTask));
            }

            break;
    } // switch

    if(DEBUG_TASKMANAGER) {
        std::cout << "======= TaskManager::executeCurrentTask =======\n";
        std::cout << "task type: " << taskTypeToString(task_queue.top()->getTaskType()) << "\n";
        std::cout << "task status: " << statusToString(task_queue.top()->getStatus()) << "\n";
        std::cout << "================================================\n" << std::endl;
    }
}


void TaskManager::addTask(std::unique_ptr<Task> task) 
{
    if(task != nullptr){
        if(DEBUG_TASKMANAGER) {
            std::cout << "\n======= TaskManager::addTask ========\n";
            std::cout << "tasktype: " << taskTypeToString(task->getTaskType()) << std::endl;
            std::cout << "=====================================\n";
        }
        task_queue.push(std::move(task));
    }
    else {
        std::cout << "failed to push to queue\n" << std::endl;
    }
}

/*
    Read tasks from a json file. All tasks in the file are assumed to be in order.
*/
void TaskManager::importTasksFromJSON(std::string filename)
{
    if(DEBUG_TASKMANAGER) std::cout << "======== TaskManager::importTasksFromJson ========" << std::endl;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(filename, pt);
    std::vector<std::unique_ptr<Task>> task_vector;
    double endpoint_x;
    double endpoint_y;
    double endpoint_orientation;
    bool endpoint_orientation_required;

    for (const auto& taskkey : pt) {        
        endpoint_x = taskkey.second.get_child("endpoint").get_child("x").get_value<double>();
        endpoint_y = taskkey.second.get_child("endpoint").get_child("y").get_value<double>();
        endpoint_orientation = taskkey.second.get_child("endpoint").get_child("yaw").get_value<double>();
        endpoint_orientation_required = taskkey.second.get_child("endpoint_settings").get_child("orientation_required").get_value<bool>();

        switch(taskTypeToEnum(taskkey.second.get_child("type").get_value<std::string>())) {
            case NAVIGATETO:
                std::unique_ptr<NavigateToTask> task = std::make_unique<NavigateToTask>(endpoint_orientation, endpoint_orientation_required);
                task->setEndpoint(endpoint_x, endpoint_y, endpoint_orientation);
                task_vector.push_back(std::move(task));
                break;
        }
    }

    // insert tasks into task manager queue
    for(std::vector<std::unique_ptr<Task>>::reverse_iterator v = task_vector.rbegin(); v != task_vector.rend(); ++v){
        switch((*v)->getTaskType()){
            case NAVIGATETO:
                addTask(std::move(*v));
                break;
            case PATHCORRECTION:
                break;
            case NA:
            default:
                break;
        }
    }

    if(DEBUG_TASKMANAGER) std::cout << "====== end TaskManager::importTasksFromJson ========" << std::endl;
}

std::unique_ptr<Task> TaskManager::taskFactory(TaskType ttype)
{
    if(DEBUG_TASKMANAGER) {
        std::cout << "======= TaskManager::taskFactory ========\n";
        std::cout << "taskType: " << taskTypeToString(ttype) << "\n";
        std::cout << "=========================================\n" << std::endl;
    }
    std::unique_ptr<Task> task;
    switch(ttype){
        case NAVIGATETO:
            task = std::make_unique<NavigateToTask>();
            return task;
            break;
        case PATHCORRECTION:
            return std::make_unique<NavigateToTask>();
            break;
        case NA:
        default:
            return nullptr;
            //task.reset(nullptr);
            return task;
            break;
    }
}
    