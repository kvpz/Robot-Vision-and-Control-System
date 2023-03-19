#include "taskmanager.hpp"
#include "travelTask.hpp"
#include "correctionTask.hpp"
#include "orientTask.hpp"

TaskManager::TaskManager() 
{ }

/*
    robot info required to be passed to all notStarted:
    - current x and y position
    - angle to destination
    
    robot info required to be passed to all inProgress:
    - current x and y position
    - 
*/
void TaskManager::executeCurrentTask(Map* map, Navigator* navigator) 
{
    switch(task_queue.top().getStatus()) {
        case NOTSTARTED:
            //RobotState robotState;
            task_queue.top().notStarted(map, navigator, nextRobotState);
            break;  
            
        case INPROGRESS:
            task_queue.top().inProgress(map, navigator, nextRobotState);
            //ROBOTASKS::TaskOperations::travel_task_updater(robot, task, nextRobotState);
            break;

        case SUSPENDED:
            task_queue.top().suspended();
            if(task_queue.top().getTaskType() == TRAVEL){
                Task newTask(CORRECTPATH);
                newTask.setEndpoint(task_queue.top().getDestination().getX(), task_queue.top().getDestination().getY(), task_queue.top().getEndpointOrientation());
                task_queue.push(newTask);
            }
            break;

        case COMPLETE:
            task_queue.top().complete();

            if(task_queue.top().getTaskType() == CORRECTPATH){
                task_queue.top().setStatus(INPROGRESS);
                task_queue.pop();
                nextRobotState = STOP;
            }
            else if(task_queue.top().getTaskType() == TRAVEL) {
                task_queue.pop();
                // travelTaskCompleteState(task) definition lines below
                Task newTask(ORIENT);
                newTask.setEndpoint(task_queue.top().getDestination().getX(), task_queue.top().getDestination().getY(), task_queue.top().getEndpointOrientation());
                task_queue.push(newTask);
            }
            else if(task_queue.top().getTaskType() == ORIENT) {
                nextRobotState = STOP;
                task_queue.pop();
                task_queue.top().setStatus(INPROGRESS);
            }
            else{
                task_queue.pop();
            }


            break;
    }
}


void TaskManager::addTask(Task& task) 
{
    task_queue.push(task);
}

RobotState TaskManager::getNextRobotState() 
{
    return nextRobotState;
}

/*
    Read tasks from a json file. All tasks in the file are assumed to be in order.
*/
void TaskManager::importTasksFromJSON()
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_json("tasks.json", pt);
    std::vector<Task> task_vector;

    for (const auto& taskkey : pt) {
        Task* task = taskFactory(taskTypeToEnum(taskkey.second.get_child("type").get_value<std::string>()));
        //Task task(taskTypeToEnum(taskkey.second.get_child("type").get_value<std::string>()));
        double endpoint_x = taskkey.second.get_child("endpoint").get_child("x").get_value<double>();
        double endpoint_y = taskkey.second.get_child("endpoint").get_child("y").get_value<double>();
        double endpoint_orientation = taskkey.second.get_child("endpoint").get_child("yaw").get_value<double>();
        double endpoint_orientation_required = taskTypeToEnum(taskkey.second.get_child("endpoint_settings").get_value<std::string>());
        task->setEndpoint(endpoint_x, endpoint_y, endpoint_orientation);
        task_vector.push_back(*task);
    }

    // insert tasks 
    for(auto v = task_vector.rbegin(); v != task_vector.rend(); ++v){
        addTask(*v);
        //task_queue.push(*v);
        //printTaskInfo(task_queue.top());
    }
}

bool TaskManager::hasTasks()
{
    return task_queue.empty();
}

Task* TaskManager::taskFactory(TaskType ttype)
{
    switch(ttype){
        case TRAVEL:
            return new TravelTask();
            break;
        case CORRECTPATH:
            return new CorrectionTask();
            break;
        case ORIENT: 
            return new OrientTask();
            break;
        case NA:
        default:
            return nullptr;
            break;
    }
}
    