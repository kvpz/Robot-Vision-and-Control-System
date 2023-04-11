#ifndef TASKIMPORTER_HPP
#define TASKIMPORTER_HPP
#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <mqueue.h>
#include <set>
#include <condition_variable>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "task.hpp"
#include "enums/tasktype.hpp"
#include "enums/robotState.hpp"
#include "map.hpp"
#include "settings.hpp"
#include "navigatetotask.hpp"
#include "enums/mandibles.hpp"
#include "attractioncolortask.hpp"
#include "dropchiptask.hpp"
#include "objectmappingtask.hpp"

class TaskImporter
{
public:

    static void importTasksFromJSON(std::string filename, 
                auto& low_priority_tasks, auto& high_priority_tasks)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(filename, pt);

        for (const auto& taskkey : pt) {
            TaskType taskType = taskTypeToEnum(taskkey.second.get_child("type").get_value<std::string>());

            if(taskType == NAVIGATETO) {
                parseNavigateToTask(taskkey, low_priority_tasks, high_priority_tasks);
            }
            else if(taskType == DROPCHIP) {        
                parseDropChipTask(taskkey, low_priority_tasks, high_priority_tasks);                
            }
            else if(taskType == ATTRACTIONCOLOR) {
                parseAttractionColor(taskkey, low_priority_tasks, high_priority_tasks);
            }
        }

        if(DEBUG_TASKMANAGER) std::cout << "======== TaskManager::importTasksFromJson ========" << std::endl;
        if(DEBUG_TASKMANAGER) std::cout << "====== end TaskManager::importTasksFromJson ========" << std::endl;
    }

private:
    // navigateTo task JSON keys
    static double endpoint_x;
    static double endpoint_y;
    static double endpoint_orientation;
    static bool endpoint_orientation_required;
    static TravelDirection travelDirection;
    
    // attraction color detection task JSON keys
    static std::string mqname;
    static int priority;
    static std::string startTime;
    
    static void parseNavigateToTask(const auto& taskkey, 
                auto& low_priority_tasks, auto& high_priority_tasks) 
    {
        // parse the json object for navigateto task
        endpoint_x = taskkey.second.get_child("endpoint").get_child("x").get_value<double>();
        endpoint_y = taskkey.second.get_child("endpoint").get_child("y").get_value<double>();
        endpoint_orientation = taskkey.second.get_child("endpoint").get_child("yaw").get_value<double>();
        endpoint_orientation_required = taskkey.second.get_child("endpoint_settings").get_child("orientation_required").get_value<bool>();
        travelDirection = stringToTravelDirection(taskkey.second.get_child("move_direction").get_value<std::string>());
        XYPoint xypoint(endpoint_x, endpoint_y);

        try {
            startTime = taskkey.second.get_child("start_time").get_value<std::string>();
        }
        catch(boost::property_tree::ptree_bad_path& e){
            std::cout << "[ERROR] start_time node not found" << std::endl;
            startTime = "";
        }

        if(startTime == "now") {
            // add to high priority queue
            high_priority_tasks.insert(std::make_unique<NavigateToTask>(xypoint, 
                                            endpoint_orientation, 
                                            endpoint_orientation_required,
                                            travelDirection));
        }
        else {
            // add to lower priority queue
            low_priority_tasks.emplace_back(std::make_unique<NavigateToTask>(xypoint, 
                                            endpoint_orientation, 
                                            endpoint_orientation_required,
                                            travelDirection));
        }
    }

    static void parseDropChipTask(const auto& taskkey, 
                auto& low_priority_tasks, auto& high_priority_tasks)
    {
        // parse the json object for navigateto task
        endpoint_x = taskkey.second.get_child("endpoint").get_child("x").get_value<double>();
        endpoint_y = taskkey.second.get_child("endpoint").get_child("y").get_value<double>();
        endpoint_orientation = taskkey.second.get_child("endpoint").get_child("yaw").get_value<double>();
        endpoint_orientation_required = taskkey.second.get_child("endpoint_settings").get_child("orientation_required").get_value<bool>();
        travelDirection = stringToTravelDirection(taskkey.second.get_child("move_direction").get_value<std::string>());
        XYPoint xypoint(endpoint_x, endpoint_y);
        low_priority_tasks.emplace_back(std::make_unique<DropChipTask>(xypoint, 
                                        endpoint_orientation, 
                                        endpoint_orientation_required));
    }

    static void parseAttractionColor(const auto& taskkey, 
                auto& low_priority_tasks, auto& high_priority_tasks)
    {
        // In the future the attraction color detection task type can 
        // contain info about the following:
        // (1) approximate location of the attractions
        // (2) region of pixels to observe from the video frame
        mqname = taskkey.second.get_child("mqname").get_value<std::string>();
        priority = taskkey.second.get_child("priority").get_value<int>();
        try {
            startTime = taskkey.second.get_child("start_time").get_value<std::string>();
        }
        catch(boost::property_tree::ptree_bad_path& e){
            std::cout << "[ERROR] start_time node not found" << std::endl;
            startTime = "";
        }

        if(startTime == "now") {
            high_priority_tasks.insert(std::make_unique<AttractionColorTask>(mqname.c_str()));
        }
        else {
            low_priority_tasks.emplace_back(std::make_unique<AttractionColorTask>(mqname.c_str()));
        }
    }
};

#endif