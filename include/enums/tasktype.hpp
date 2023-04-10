#ifndef TASKTYPE_HPP
#define TASKTYPE_HPP

enum TaskType {
        NAVIGATETO, DROPCHIP, GRASP, STACKPED,
        PATHCORRECTION, POSECORRECTION, ATTRACTIONCOLOR, 
        PICKUPOBJECT, FOLLOWOBJECT, OBJECTSEARCH, CONTROLMANDIBLES, 
        CONTROLWINGS, NA
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
  case PICKUPOBJECT:
    return "pickupobject";
  case FOLLOWOBJECT:
    return "followobject";
  case OBJECTSEARCH:
    return "objectsearch";
  case CONTROLMANDIBLES:
    return "controlmandibles";
  case CONTROLWINGS:
    return "controlwings";
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
    else if(ttype == "pickupobject") return PICKUPOBJECT;
    else if(ttype == "followobject") return FOLLOWOBJECT;
    else if(ttype == "objectsearch") return OBJECTSEARCH;
    else if(ttype == "controlmandibles") return CONTROLMANDIBLES;
    else if(ttype == "controlwings") return CONTROLWINGS;
    else return NA;
}

 #endif
