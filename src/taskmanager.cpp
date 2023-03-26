#include "taskmanager.hpp"
#include "navigatetotask.hpp"
#include "pathcorrectiontask.hpp"
#include "posecorrectiontask.hpp"
#include "dropchiptask.hpp"

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
void TaskManager::executeCurrentTask(std::shared_ptr<Map> map, 
                                     std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    TaskType nextTaskType = NA;
    std::unique_ptr<Task> newTask;
    
    switch(task_queue.top()->getStatus()) {
        case TaskStatus::NOTSTARTED:
            task_queue.top()->notStarted(map, navigator, nextRobotState);
            handleNotStartedTask(map, navigator, nextRobotState, nextTaskType);
            break;   
            
        case TaskStatus::INPROGRESS:
            task_queue.top()->inProgress(map, navigator, nextRobotState);
            handleInProgressTask(map, navigator, nextRobotState, nextTaskType);
            break;

        case TaskStatus::SUSPENDED:
            task_queue.top()->suspended(map, navigator, nextRobotState, nextTaskType);
            handleSuspendedTask(map, navigator, nextRobotState, nextTaskType);

            break;

        case TaskStatus::COMPLETE:
            task_queue.top()->complete(map, navigator, nextRobotState, nextTaskType);
            handleCompletedTask(map, navigator, nextRobotState, nextTaskType);

            break;
    } // switch

    if(DEBUG_TASKMANAGER) {
        std::cout << "======= TaskManager::executeCurrentTask =======\n";
        std::cout << "task type: " << taskTypeToString(task_queue.top()->getTaskType()) << "\n";
        std::cout << "task status: " << statusToString(task_queue.top()->getStatus()) << "\n";
        std::cout << "task queue size: " << task_queue.size() << "\n";
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
    TravelDirection travelDirection;

    for (const auto& taskkey : pt) {        
        endpoint_x = taskkey.second.get_child("endpoint").get_child("x").get_value<double>();
        endpoint_y = taskkey.second.get_child("endpoint").get_child("y").get_value<double>();
        endpoint_orientation = taskkey.second.get_child("endpoint").get_child("yaw").get_value<double>();
        endpoint_orientation_required = taskkey.second.get_child("endpoint_settings").get_child("orientation_required").get_value<bool>();
        travelDirection = stringToTravelDirection(taskkey.second.get_child("move_direction").get_value<std::string>());
        XYPoint xypoint(endpoint_x, endpoint_y);

        switch(taskTypeToEnum(taskkey.second.get_child("type").get_value<std::string>())) {
            case NAVIGATETO:
                //std::unique_ptr<NavigateToTask> task = 
                task_vector.push_back(std::make_unique<NavigateToTask>(xypoint, 
                                                     endpoint_orientation, 
                                                     endpoint_orientation_required,
                                                     travelDirection));
                //task_vector.push_back(std::move(task));
                break;
            case DROPCHIP:
                //std::unique_ptr<DropChipTask> task = 
                task_vector.push_back(std::make_unique<DropChipTask>(xypoint, 
                                                   endpoint_orientation, 
                                                   endpoint_orientation_required));
                //task_vector.push_back(std::move(task));
                break;
        }
    }

    // insert tasks into task manager queue
    for(std::vector<std::unique_ptr<Task>>::reverse_iterator task = task_vector.rbegin(); task != task_vector.rend(); ++task){
        switch((*task)->getTaskType()) {
            case NAVIGATETO:
                addTask(std::move(*task));
                break;
            case DROPCHIP:
                addTask(std::move(*task));
                break;
            case PATHCORRECTION:
                std::cout << "Path correction tasks cannot be defined in task configuration JSON file." << std::endl;
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
    std::unique_ptr<Task> task;
    switch(ttype){
        case NAVIGATETO:
            task = std::make_unique<NavigateToTask>();
            break;
        case PATHCORRECTION:
            task = std::make_unique<PathCorrectionTask>();
            break;
        case NA:
        default:
            task = nullptr;
            //task.reset(nullptr);
            break;
    }

    if(DEBUG_TASKMANAGER) {
        std::cout << "======= TaskManager::taskFactory ========\n";
        std::cout << "taskType: " << taskTypeToString(ttype) << "\n";
        std::cout << "=========================================\n" << std::endl;
    }

    return task;
}

void TaskManager::scheduleNewTask(TaskType tasktype, std::shared_ptr<Map> map)
{
    std::cout << "Sheduling a new task: " << taskTypeToString(tasktype) << std::endl;
    switch(tasktype) {
        case PATHCORRECTION:
            task_queue.push(std::make_unique<PathCorrectionTask>());
            break;
        case POSECORRECTION:
            task_queue.push(std::make_unique<PoseCorrectionTask>());
            break;
        // TODO bottom never called. Test further
        case NAVIGATETO: {
            XYPoint xypoint = map->getNextDestinationXY(); 
            task_queue.push(std::make_unique<NavigateToTask>(xypoint, map->getDestinationOrientation(), map->getIsEndpointOrientationRequired(), TravelDirection::error));
            break;
        }
        default:
            std::cout << "\n[ERROR] TaskManager::executeCurrentTask. Current completed task requesting unknown task type\n" << std::endl;
            break;
    }            
}


void TaskManager::handleNotStartedTask(std::shared_ptr<Map> map, 
                                       std::shared_ptr<Navigator> navigator, 
                                       RobotState& nextRobotState, TaskType& nextTaskType)
{
}

void TaskManager::handleInProgressTask(std::shared_ptr<Map> map, 
                                       std::shared_ptr<Navigator> navigator, 
                                       RobotState& nextRobotState, TaskType& nextTaskType)
{

}

void TaskManager::handleSuspendedTask(std::shared_ptr<Map> map, 
                                       std::shared_ptr<Navigator> navigator, 
                                       RobotState& nextRobotState, TaskType& nextTaskType)
{
    if(nextTaskType != NA)
        scheduleNewTask(nextTaskType, map);
}

void TaskManager::handleCompletedTask(std::shared_ptr<Map> map, 
                                       std::shared_ptr<Navigator> navigator, 
                                       RobotState& nextRobotState, TaskType& nextTaskType)
{
    if(task_queue.top()->getTaskType() == PATHCORRECTION) {
        task_queue.pop();
        // next task after a pose or path correction task should be 
        // a navigateTo task
        if(task_queue.top()->getTaskType() == NAVIGATETO)
            task_queue.top()->setStatus(TaskStatus::INPROGRESS);
    }
    else if(task_queue.top()->getTaskType() == POSECORRECTION) {
        task_queue.pop(); // pop pose correction task
        task_queue.pop(); // pop the navigate to task that had created pose correction task
    }
    else if(nextTaskType != NA) {
        task_queue.pop();
        scheduleNewTask(nextTaskType, map);
    }
    else {
        task_queue.pop();
    }
}