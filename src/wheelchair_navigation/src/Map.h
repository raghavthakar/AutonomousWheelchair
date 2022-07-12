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

    void map_received_cb(nav_msgs::OccupancyGrid::ConstPtr map)
    {
        this->map = *map;
        ROS_INFO("Map received!\n");

        // display the map
        // for(auto i:this->map.data)
        //     std::cout<<(int)i<<" ";
    }

public:
    MapHandler()
    {
        map_subscriber = this->node_handle.subscribe("/map", 1000, &MapHandler::map_received_cb, this);
    }

    // return the map 
    nav_msgs::OccupancyGrid getMap()
    {
        return this->map;
    }
};

#endif