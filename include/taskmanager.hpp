#ifndef TASKMANAGER_HPP
#define TASKMANAGER_HPP
#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <thread>
#include <mutex>
#include <mqueue.h>
#include <set>
#include <condition_variable>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "navigator.hpp"
#include "task.hpp"
#include "enums/tasktype.hpp"
#include "enums/robotState.hpp"
#include "map.hpp"
#include "settings.hpp"
#include "enums/mandibles.hpp"

//#include "taskimporter.hpp"

class Task;
/*
    ITaskManager is an interface class.
    This class contains the minimum amount of methods required
    to implement a TaskManager for a specific type of task. 
    The reason why this interface (abstract) class exists is because there
    are several functions that all tasks have in common depending on the state
    of the task. All tasks can be in the following state:
    (1) not started
    (2) in progress
    (3) suspended
    (4) completed

    When a task is in one of these states, the robot state machine and task scheduler 
    must be in a specific state dependent on the task. The task manager must control the
    robot's state depending on the current task's state. The robot's state can also have
    an effect on the state of the current task. 
*/

struct TaskComparator {
    bool operator()(const std::unique_ptr<Task>& a, const std::unique_ptr<Task>& b) const {
        return (a->getPriority() < b->getPriority());
    }
};

class TaskManager
{

public:
    TaskManager(){};

    void executeCurrentTask(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, 
                            std::shared_ptr<VisionData> visionData,
                            std::vector<RobotState>& nextRobotStates);

    void addTask(std::unique_ptr<Task>);

    void importTasksFromJSON(std::string filename);

    inline bool hasTasks() { return !high_priority_tasks.empty() || !low_priority_tasks.empty(); }

    void scheduleNewTask(TaskType ttype, std::shared_ptr<Map> map);

    TaskType getCurrentTaskType() {
        //for(auto i = high_priority_tasks.begin(); i != high_priority_tasks.end(); ++i) {
        //    if(i->getTaskType() == )
        //}
        //return high_priority_tasks//*task_queue.top();
        return currentTaskType;
    }

    TaskStatus getCurrentTaskStatus() {
        return currentTaskStatus;
    }

    std::vector<Task> getHighPriorityTasks()
    {
        std::vector<Task> tasks;
        for(auto i = high_priority_tasks.begin(); i != high_priority_tasks.end(); ++i) {
            tasks.push_back(*(*i));
        }
        return tasks;
    }

    void printHighPriorityTasks();

private:
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;

    std::unique_ptr<Task> taskFactory(TaskType ttype);

    std::multiset<std::unique_ptr<Task>, TaskComparator> high_priority_tasks;

    std::deque<std::unique_ptr<Task>> low_priority_tasks;

    std::vector<std::unique_ptr<Task>> pending_high_priority_tasks;

    unsigned int currentTaskPriority;
    TaskType currentTaskType;
    TaskStatus currentTaskStatus;
    
    void handleNotStartedTask(std::shared_ptr<Map> map, 
                              std::shared_ptr<Navigator> navigator, 
                              RobotState& nextRobotState, TaskType& nextTaskType);
    void handleInProgressTask(std::shared_ptr<Map> map, 
                              std::shared_ptr<Navigator> navigator, 
                              RobotState& nextRobotState, TaskType& nextTaskType);
    void handleSuspendedTask(std::shared_ptr<Map> map, 
                             std::shared_ptr<Navigator> navigator, 
                             RobotState& nextRobotState, TaskType& nextTaskType);
    void handleCompletedTask(std::shared_ptr<Map> map, 
                             std::shared_ptr<Navigator> navigator, 
                             RobotState& nextRobotState, TaskType& nextTaskType);

    void parseNavigateToTask(boost::property_tree::ptree::value_type taskkey);
    void parseDropChipTask(boost::property_tree::ptree::value_type taskkey);
    void parseAttractionColorTask(boost::property_tree::ptree::value_type taskkey);
    void parseControlMandiblesTask(boost::property_tree::ptree::value_type taskkey);
    void parseControlWingsTask(boost::property_tree::ptree::value_type taskkey);
    void parseFollowObjectTask(boost::property_tree::ptree::value_type taskkey);
    template<class T> 
    void enqueueTask(std::unique_ptr<T> task, std::string startTime);
};



#endif