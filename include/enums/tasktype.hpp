#ifndef TASKTYPE_HPP
#define TASKTYPE_HPP

enum TaskType {
        NAVIGATETO, DROPPAYLOAD, GRASP, STACKPED,
        PATHCORRECTION, POSECORRECTION, NA
};

static std::string taskTypeToString(TaskType ttype) 
{
  switch(ttype) {
  case NAVIGATETO:
    return "navigateto";
  case PATHCORRECTION:
    return "pathcorrection";
  case POSECORRECTION:
    return "posecorrection";
  case DROPPAYLOAD:
    return "droppayload";
  case GRASP:
    return "grasp";
  case STACKPED:
    return "stackped";
  case NA:
    return "NA";
  default:
    return "error";
  }
}

static TaskType taskTypeToEnum(std::string ttype) 
{
    if(ttype == "navigateto") return NAVIGATETO;
    else if(ttype == "pathcorrection") return PATHCORRECTION;
    else if(ttype == "posecorrection") return POSECORRECTION;
    else if(ttype == "droppayload") return DROPPAYLOAD;
    else return NA;
}

 #endif
