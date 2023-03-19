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
    TaskManager();

    void executeCurrentTask(Map* map, Navigator* navigator);


    void addTask(Task& task);

    //void importTasksFromJSON(){}

    RobotState getNextRobotState();

    /*
        importTasksFromJson

        Read tasks from a json file. All tasks in the file are assumed to be in order.
    */
    void importTasksFromJSON();

    bool hasTasks();

private:
    std::stack<Task> task_queue;
    //Task& task;
    //Robot&* robot;
    RobotState nextRobotState;

    Task* taskFactory(TaskType ttype);

};


#endif