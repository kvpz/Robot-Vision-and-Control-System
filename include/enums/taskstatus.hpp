#ifndef TASKSTATUS_HPP
#define TASKSTATUS_HPP
#include <string>

enum class TaskStatus:char {
    NOTSTARTED, COMPLETE, INPROGRESS, SUSPENDED
};

static std::string statusToString(TaskStatus status) 
{
    switch(status) {
        case TaskStatus::NOTSTARTED:
        return "NOTSTARTED";
        case TaskStatus::COMPLETE:
        return "COMPLETE";
        case TaskStatus::INPROGRESS:
        return "INPROGRESS";
        case TaskStatus::SUSPENDED:
        return "SUSPENDED";
        default:
        return "ERROR";
    }
}

#endif