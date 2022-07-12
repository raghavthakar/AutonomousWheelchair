#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <cstdlib>
#include <list>
#include <random>
#include <nav_msgs/OccupancyGrid.h>

// struct used to represent all points
struct Point
{
    unsigned int x;
    unsigned int y;
};

class Node
{
    Point coords;
    std::list<std::shared_ptr<Node>> children; //maintains a list of all child connections
    std::shared_ptr<Node> parent; //stores a reference to the parent of the node

public:
    void setCoords(int x, int y)
    {
        coords.x = x;
        coords.y = y;
    }

    void display()
    {
        ROS_INFO("Random config::X:%d Y:%d", coords.x, coords.y);
    }
};

class RRTHandler
{
    nav_msgs::OccupancyGrid map; //store the map object
    unsigned int map_dim; //store the dimension of the map for reference
    Point start_point;
    Point target_point;
    unsigned int step_length; //the maximum distance (in pixels) between connected nodes

    // generates a random configuration within the workspace and returns it as a Point
    Node randomConfig()
    {
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist6(0,map_dim-1); // distribution in range
        
        Node random_config;
        random_config.setCoords(dist6(rng), dist6(rng));
        return random_config;
    }

public:
    RRTHandler(nav_msgs::OccupancyGrid map, unsigned int map_dim)
    {
        this->map = map;
        this->map_dim = map_dim;

        // set default start and target values
        this->start_point.x = 0;
        this->start_point.y = 0;
        this->target_point.x = map_dim-1;
        this->target_point.y = map_dim-1;

        // set default step length value
        this->step_length = map_dim/25;
    }

    void setStartPoint(unsigned int x, unsigned int y)
    {
        if(x<this->map_dim && x>0)
            this->start_point.x = x;
        if(y<this->map_dim && y>0)
            this->start_point.y = y;
    }

    void setTargetPoint(unsigned int x, unsigned int y)
    {
        if(x<this->map_dim && x>0)
            this->target_point.x = x;
        if(y<this->map_dim && y>0)
            this->target_point.y = y;
    }

    void setStepLength(unsigned int step_length)
    {
        this->step_length = step_length;
    }

    void RRT(unsigned int num_iterations)
    {
        unsigned int iteration_count = 0; //keeps track of the number of iterations that have taken place
        Node q_rand; //random configuration sampled in the workspace
        Node q_near; //node in the tree that is closest to q_rand
        Node q_new; //the configuration that is withing step_length of the q_near
        while(iteration_count < num_iterations)
        {
            q_rand = randomConfig();
            q_rand.display();
            iteration_count++;
        }
    }
};

#endif