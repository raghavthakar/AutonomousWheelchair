#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>

class MapHandler
{
    ros::NodeHandle node_handle;
    ros::Subscriber map_subscriber;
    nav_msgs::OccupancyGrid map;

public:
    MapHandler() { }

    // return the map 
    nav_msgs::OccupancyGrid getMap()
    {
        this->map = *ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", this->node_handle); //wait to receive the map
        return this->map;
    }
};

#endif