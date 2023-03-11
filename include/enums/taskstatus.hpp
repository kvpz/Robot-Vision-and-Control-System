 #ifndef TASKSTATUS_HPP
 #define TASKSTATUS_HPP
#include <string>

enum Status {
    NOTSTARTED, COMPLETE, INPROGRESS, SUSPENDED
};

static std::string statusToString(Status status) 
{
    switch(status) {
        case NOTSTARTED:
        return "NOTSTARTED";
        case COMPLETE:
        return "COMPLETE";
        case INPROGRESS:
        return "INPROGRESS";
        case SUSPENDED:
        return "SUSPENDED";
        default:
        return "ERROR";
    }
}

  #endif