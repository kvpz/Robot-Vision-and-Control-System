#ifndef TASKMANAGER_HPP
#define TASKMANAGER_HPP
#include <iostream>
#include <stack>
#include <vector>
#include <utility>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "task.hpp"
#include "enums/tasktype.hpp"
#include "enums/robotState.hpp"
#include "map.hpp"
#include "navigator.hpp"

#define DEBUG_TASKMANAGER true

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

    void executeCurrentTask(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator, RobotState& robotState);
    //RobotState executeCurrentTask(std::shared_ptr<Map> map, std::shared_ptr<Navigator> navigator);

    void addTask(std::unique_ptr<Task>);

    void importTasksFromJSON(std::string filename);

    inline bool hasTasks() { return !task_queue.empty(); }

private:
    std::stack<std::unique_ptr<Task>> task_queue;
    //std::stack<Task> task_queue;
    std::unique_ptr<Task> taskFactory(TaskType ttype);
    //RobotState nextRobotState;
};


#endif