#ifndef TASKMANAGER_HPP
#define TASKMANAGER_HPP
#include <iostream>
#include <stack>
#include <vector>
#include <queue>
#include <utility>
#include <thread>
#include <mutex>
#include <mqueue.h>
#include <condition_variable>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "navigator.hpp"
#include "task.hpp"
#include "enums/tasktype.hpp"
#include "enums/robotState.hpp"
#include "map.hpp"


#define DEBUG_TASKMANAGER false

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
class TaskManager
{

public:
    TaskManager(){};

    void executeCurrentTask(std::shared_ptr<Map> map, 
                            std::shared_ptr<Navigator> navigator, RobotState& robotState);

    void addTask(std::unique_ptr<Task>);

    void importTasksFromJSON(std::string filename);

    inline bool hasTasks() { return !task_queue.empty(); }

    void scheduleNewTask(TaskType ttype, std::shared_ptr<Map> map);

    Task getCurrentTask() {
        return *task_queue.top();
    }

private:
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;

    std::stack<std::unique_ptr<Task>> task_queue;
    std::unique_ptr<Task> taskFactory(TaskType ttype);

/*
    std::priority_queue<std::unique_ptr<Task>, std::vector<std::unique_ptr<Task>>, 
        decltype([](const auto& a, const auto& b) { return (a->priority() > b->priority()); })> 
        high_priority_tasks_;
*/
    std::deque<std::unique_ptr<Task>> low_priority_tasks_;

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
};


#endif