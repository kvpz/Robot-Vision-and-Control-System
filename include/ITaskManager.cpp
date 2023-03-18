#include "task.hpp"

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
class ITaskManager
{
private:
    Task task;

public:
    ITaskManager() 
    {

    }

    virtual void notStarted() = 0;
    virtual void inProgress() = 0;
    virtual void suspended() = 0;
    virtual void complete() = 0;
}