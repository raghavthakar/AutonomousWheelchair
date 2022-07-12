#include "Map.h"
#include "RRT.h"
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_distance_server");

    MapHandler map_handler;
    RRTHandler rrt_handler(map_handler.getMap(), 128);
    rrt_handler.setTargetPoint(127, 127);
    rrt_handler.RRT(10);
    ros::spin();
    return 0;
}