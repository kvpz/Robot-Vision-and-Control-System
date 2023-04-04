#ifndef TASKTYPE_HPP
#define TASKTYPE_HPP

enum TaskType {
        NAVIGATETO, DROPCHIP, GRASP, STACKPED,
        PATHCORRECTION, POSECORRECTION, ATTRACTIONCOLOR, NA
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
  case DROPCHIP:
    return "dropchip";
  case GRASP:
    return "grasp";
  case STACKPED:
    return "stackped";
  case ATTRACTIONCOLOR:
    return "attractioncolor";
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
    else if(ttype == "dropchip") return DROPCHIP;
    else if(ttype == "attractioncolor") return ATTRACTIONCOLOR;
    else return NA;
}

 #endif
