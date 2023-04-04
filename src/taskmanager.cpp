#include "taskmanager.hpp"
#include "navigatetotask.hpp"
#include "pathcorrectiontask.hpp"
#include "posecorrectiontask.hpp"
#include "dropchiptask.hpp"
#include "attractioncolortask.hpp"

/*
    function: void TaskManager::executeCurrentTask

    This function should avoid implementing task specific behaviors. All actions
    or updates needed to be performed by a task should be done in their respective state functions. 

    Not all tasks may achieve all status. 
*/
void TaskManager::executeCurrentTask(std::shared_ptr<Map> map, 
                                     std::shared_ptr<Navigator> navigator, RobotState& nextRobotState) 
{
    TaskType nextTaskType = NA;
    //std::vector<std::unique_ptr<Task>> tasksToDelete;

    for(auto& task : high_priority_tasks) { 
        // read obj detection data from message queue
        currentTaskPriority = task->getPriority();
        currentTaskType = task->getTaskType();
        currentTaskStatus = task->getStatus();

        switch(task->getStatus()) {
            case TaskStatus::NOTSTARTED:
                task->notStarted(map, navigator, nextRobotState);
                handleNotStartedTask(map, navigator, nextRobotState, nextTaskType);
                break;   
                
            case TaskStatus::INPROGRESS:
                task->inProgress(map, navigator, nextRobotState);
                handleInProgressTask(map, navigator, nextRobotState, nextTaskType);
                break;

            case TaskStatus::SUSPENDED:
                task->suspended(map, navigator, nextRobotState, nextTaskType);
                handleSuspendedTask(map, navigator, nextRobotState, nextTaskType);
                break;

            case TaskStatus::COMPLETE:
                task->complete(map, navigator, nextRobotState, nextTaskType);
                handleCompletedTask(map, navigator, nextRobotState, nextTaskType);
                //tasksToDelete.push_back(std::move(task));
                break;
        } // switch

        // issue: "task" may have been deleted from multiset by this point
        //currentTaskPriority = task->getPriority();
        //currentTaskType = task->getTaskType();
        //currentTaskStatus = task->getStatus();
    }

    // delete all completed tasks from the high priority list

    for(auto i = high_priority_tasks.begin(); i != high_priority_tasks.end(); ) {
        if((*i)->getStatus() == TaskStatus::COMPLETE) {
            i = high_priority_tasks.erase(i);
        }
        else {
            ++i;
        }
    }

    if(DEBUG_TASKMANAGER) {
        std::cout << "======= TaskManager::executeCurrentTask =======\n";
        //std::cout << "task type: " << taskTypeToString(task_queue.top()->getTaskType()) << "\n";
        //std::cout << "task status: " << statusToString(task_queue.top()->getStatus()) << "\n";
        //std::cout << "task queue size: " << task_queue.size() << "\n";
        std::cout << "task type: " << taskTypeToString(getCurrentTaskType()) << "\n";
        std::cout << "task status: " << statusToString(getCurrentTaskStatus()) << "\n";
        std::cout << "task queue size: " << high_priority_tasks.size() + low_priority_tasks.size() << "\n";
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
    }
}

/*
    Read tasks from a json file. All tasks in the file are assumed to be in order.
*/
void TaskManager::importTasksFromJSON(std::string filename)
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(filename, pt);
    //std::vector<std::unique_ptr<Task>> task_vector;
    
    // navigateTo task JSON keys
    double endpoint_x;
    double endpoint_y;
    double endpoint_orientation;
    bool endpoint_orientation_required;
    TravelDirection travelDirection;
    
    // attraction color detection task JSON keys
    std::string mqname;
    int priority;
    std::string startTime;

    for (const auto& taskkey : pt) {
        TaskType taskType = taskTypeToEnum(taskkey.second.get_child("type").get_value<std::string>());

        if(taskType == NAVIGATETO) {
            // parse the json object for navigateto task
            endpoint_x = taskkey.second.get_child("endpoint").get_child("x").get_value<double>();
            endpoint_y = taskkey.second.get_child("endpoint").get_child("y").get_value<double>();
            endpoint_orientation = taskkey.second.get_child("endpoint").get_child("yaw").get_value<double>();
            endpoint_orientation_required = taskkey.second.get_child("endpoint_settings").get_child("orientation_required").get_value<bool>();
            travelDirection = stringToTravelDirection(taskkey.second.get_child("move_direction").get_value<std::string>());
            XYPoint xypoint(endpoint_x, endpoint_y);

            try {
                startTime = taskkey.second.get_child("start_time").get_value<std::string>();
            }
            catch(boost::property_tree::ptree_bad_path& e){
                std::cout << "[ERROR] start_time node not found" << std::endl;
                startTime = "";
            }

            if(startTime == "now") {
                // add to high priority queue
                high_priority_tasks.insert(std::make_unique<NavigateToTask>(xypoint, 
                                                endpoint_orientation, 
                                                endpoint_orientation_required,
                                                travelDirection));
            }
            else {
                // add to lower priority queue
                low_priority_tasks.emplace_back(std::make_unique<NavigateToTask>(xypoint, 
                                                endpoint_orientation, 
                                                endpoint_orientation_required,
                                                travelDirection));
            }
        }
        else if(taskType == DROPCHIP) {        
            // parse the json object for navigateto task
            endpoint_x = taskkey.second.get_child("endpoint").get_child("x").get_value<double>();
            endpoint_y = taskkey.second.get_child("endpoint").get_child("y").get_value<double>();
            endpoint_orientation = taskkey.second.get_child("endpoint").get_child("yaw").get_value<double>();
            endpoint_orientation_required = taskkey.second.get_child("endpoint_settings").get_child("orientation_required").get_value<bool>();
            travelDirection = stringToTravelDirection(taskkey.second.get_child("move_direction").get_value<std::string>());
            XYPoint xypoint(endpoint_x, endpoint_y);
            low_priority_tasks.emplace_back(std::make_unique<DropChipTask>(xypoint, 
                                            endpoint_orientation, 
                                            endpoint_orientation_required));
            
        }
        else if(taskType == ATTRACTIONCOLOR) {
            // In the future the attraction color detection task type can 
            // contain info about the following:
            // (1) approximate location of the attractions
            // (2) region of pixels to observe from the video frame
            mqname = taskkey.second.get_child("mqname").get_value<std::string>();
            priority = taskkey.second.get_child("priority").get_value<int>();
            try {
                startTime = taskkey.second.get_child("start_time").get_value<std::string>();
            }
            catch(boost::property_tree::ptree_bad_path& e){
                std::cout << "[ERROR] start_time node not found" << std::endl;
            }

            if(startTime == "now") {
                high_priority_tasks.insert(std::make_unique<AttractionColorTask>(mqname.c_str()));
            }
            else {
                low_priority_tasks.emplace_back(std::make_unique<AttractionColorTask>(mqname.c_str()));
            }
        }
    }

    if(DEBUG_TASKMANAGER) std::cout << "======== TaskManager::importTasksFromJson ========" << std::endl;
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

/*
    This function should schedule a new high priority task. 
    Tasks are not added to the low priority list
    because that would require more computation to have to fetch
    something from the low priority list. 
    
    Scenarios when new tasks are created (current as of April 2 2023):
    (1) when a navigate to task needs a path correction
    (2) when a navigate to task needs a pose correction
    
*/
void TaskManager::scheduleNewTask(TaskType tasktype, std::shared_ptr<Map> map)
{
    if(DEBUG_TASKMANAGER)
        std::cout << "scheduling new task " << taskTypeToString(tasktype) << std::endl;

    switch(tasktype) {
        case PATHCORRECTION:
            high_priority_tasks.insert(std::make_unique<PathCorrectionTask>());
            break;
        case POSECORRECTION:
            high_priority_tasks.insert(std::make_unique<PoseCorrectionTask>());
            break;
        case NAVIGATETO:
            high_priority_tasks.insert(std::make_unique<NavigateToTask>(map->getNextDestinationXY(), map->getDestinationOrientation(), map->getIsEndpointOrientationRequired(), TravelDirection::error));
            break;
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
    std::cout << "handling suspended task " << taskTypeToString(currentTaskType) << std::endl;
    if(nextTaskType != NA)
        scheduleNewTask(nextTaskType, map);
}

void TaskManager::handleCompletedTask(std::shared_ptr<Map> map, 
                                       std::shared_ptr<Navigator> navigator, 
                                       RobotState& nextRobotState, TaskType& nextTaskType)
{
    // get current task from the high priority queue
    // using the current task id stored in an instance of this class
    //auto const [pqbegin, pqend] = high_priority_tasks.equal_range(currentTaskPriority);

    for(auto i = high_priority_tasks.begin(); i != high_priority_tasks.end(); ++i) {
        if((*i)->getPriority() != currentTaskPriority)
            continue;
        if((*i)->getTaskType() != currentTaskType)
            continue;
        // current task has been found

        std::cout << "handleCompleted Task : " << taskTypeToString((*i)->getTaskType()) << std::endl;
        if((*i)->getTaskType() == PATHCORRECTION) {
            // find the navigate to task in the high priority queue and change
            // its status to IN PROGRESS from SUSPENDED
            for(auto pqi = high_priority_tasks.begin(); pqi != high_priority_tasks.end(); ++pqi) {
                if((*pqi)->getTaskType() == NAVIGATETO) {
                    std::cout << "found next task NAVIGATETO" << std::endl;
                    // next task after a pose or path correction task should be 
                    // a navigateTo task
                    (*pqi)->setStatus(TaskStatus::INPROGRESS);
                    // pop the current task
                    std::cout << "before erase" << std::endl;
                    //high_priority_tasks.erase(i);
                    std::cout << "after erase" << std::endl;
                    break;
                }
            }
        }
        else if((*i)->getTaskType() == POSECORRECTION) {
            // pop pose correction task
            high_priority_tasks.erase(i);
            // pop the navigate to task that had created pose correction task
            for(auto pqi = high_priority_tasks.begin(); pqi != high_priority_tasks.end(); ++pqi) {
                if((*pqi)->getTaskType() == NAVIGATETO) {
                    high_priority_tasks.erase(pqi);
                    break;
                }
            }
        }
        else if(nextTaskType != NA) {
            high_priority_tasks.erase(i);
            //task_queue.pop();
            scheduleNewTask(nextTaskType, map);
        }
        else {
            high_priority_tasks.erase(i);
            //task_queue.pop();
        }
    }
}

void TaskManager::printHighPriorityTasks() 
{
    for(auto pqi = high_priority_tasks.begin(); pqi != high_priority_tasks.end(); ++pqi) {
        (*pqi)->printTaskInfo();
    }
}

/*
    Note: we cannot have two high priority tasks that change the state
    of the robot at the same time. Therefore, make sure that the in progress
    tasks in the high priority task list do change the state of the robot. 

    There should only be one instance of a task type in the priority queue
*/