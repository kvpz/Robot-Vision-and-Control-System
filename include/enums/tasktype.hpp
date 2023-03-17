#ifndef TASKTYPE_HPP
#define TASKTYPE_HPP

enum TaskType {
        TRAVEL, DROPPAYLOAD, GRASP, STACKPED,
        CORRECTPATH, ORIENT, NA
};

static std::string taskTypeToString(TaskType ttype) 
{
  switch(ttype) {
  case TRAVEL:
    return "travel";
  case CORRECTPATH:
    return "correctpath";
  case ORIENT:
    return "orient";
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
    if(ttype == "travel") return TRAVEL;
    else if(ttype == "correctpath") return CORRECTPATH;
    else if(ttype == "orient") return ORIENT;
    else if(ttype == "droppayload") return DROPPAYLOAD;
    else return NA;
}

 #endif
