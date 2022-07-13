#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <cstdlib>
#include <list>
#include <random>
#include <cmath>
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
    std::list<Node*> children; //maintains a list of all child connections
    Node* parent; //stores a reference to the parent of the node

public:
    Node(){ }

    Node(Point coords)
    {
        this->coords = coords;
    }

    Node(int x, int y)
    {
        this->coords.x = x;
        this->coords.y = y;
    }

    void setCoords(int x, int y)
    {
        coords.x = x;
        coords.y = y;
    }

    Point getCoords()
    {
        return coords;
    }

    void display()
    {
        ROS_INFO("X:%d Y:%d", coords.x, coords.y);
    }

    void addChild(Node* child_node)
    {
        this->children.push_back(child_node);
    }

    void setParent(Node* parent_node)
    {
        this->parent = parent_node;
    }

    std::list<Node*> getChildren()
    {
        return this->children;
    }
};

class RRTHandler
{
    nav_msgs::OccupancyGrid map; //store the map object
    unsigned int map_dim; //store the dimension of the map for reference
    Point start_point;
    Point target_point;
    unsigned int step_length; //the maximum distance (in pixels) between connected nodes
    std::list<Node*> all_nodes; //keep track of all nodes to easily find them 

    float distanceBetween(Node* node1, Node* node2)
    {
        return(sqrt((node1->getCoords().x-node2->getCoords().x)*(node1->getCoords().x-node2->getCoords().x)
                     + (node1->getCoords().y-node2->getCoords().y)*(node1->getCoords().y-node2->getCoords().y)));
    }

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

    // return a reference to the node that is closest to the random configuration
    Node* closestToRandomConfig(Node* q_rand)
    {
        float min_dist = -1;
        Node* q_near;
        for(Node* tree_node:this->all_nodes)
        {
            // tree_node->display();
            if(min_dist < 0)
            {
                min_dist = distanceBetween(q_rand, tree_node);
                q_near = tree_node;
            }
            else if(distanceBetween(q_rand, tree_node) < min_dist)
            {
                min_dist = distanceBetween(q_rand, tree_node);
                q_near = tree_node;
            }
            ROS_INFO("Min dist: %f", min_dist);
        }
        return q_near;
    }

    // generates a new configuration in the directio of q_rand from q_near but within step_length
    Node* newConfiguration(Node* q_rand, Node* q_near)
    {
        Node* q_new;
        float nodes_distance = distanceBetween(q_rand, q_near);
        if(nodes_distance < (float) this->step_length)
        {
            q_new = new Node(q_rand->getCoords());
        }
        else
        {
            // Compute the slope between q_rand and q_new
            float theta = atan2(q_rand->getCoords().y-q_near->getCoords().y, q_rand->getCoords().x-q_near->getCoords().x);

            // compute the new point q_new
            int new_x = q_near->getCoords().x + this->step_length * cos(theta);
            int new_y = q_near->getCoords().y + this->step_length * sin(theta);

            q_new = new Node(new_x, new_y);
        }

        return q_new;
    }

    // checks if a node is occupied or not
    bool isOccupied(Node* node)
    {
        ROS_INFO("Occupation: %d", (int) this->map.data[100]);
        return false;
    }

    // Display the whole RRT tree
    void displayRRT()
    {
        for(auto tree_node:this->all_nodes)
        {
            tree_node->display();
            ROS_INFO("Children:");
            for(auto child_node:tree_node->getChildren())
            {
                child_node->display();
            }
            ROS_INFO("--------------");
        }
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
        this->step_length = 10;
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
        
        Node* q_root = new Node(this->start_point); //starting node at the start point
        this->all_nodes.push_back(q_root); //add the starting node to list of all nodes

        while(iteration_count < num_iterations)
        {
            Node* q_rand = new Node(randomConfig()); //random configuration generated on the map
            ROS_INFO("Random config: ");
            q_rand->display();

            Node* q_near = closestToRandomConfig(q_rand); //search through all nodes to find the one closest to random configuration
            ROS_INFO("Near node: ");
            q_near->display();

            Node* q_new = newConfiguration(q_rand, q_near); //generate a new configuration along q_rand but within step length
            ROS_INFO("New Node: ");
            q_new->display();

            if(isOccupied(q_new))
            {
                iteration_count--;
                continue;
            }

            q_near->addChild(q_new); //make new node the child node of nearest node
            q_new->setParent(q_near); //make nearest node the parent of new node
            this->all_nodes.push_back(q_new); //add the new node to list of all nodes

            std::cout<<"\n";
            iteration_count++;
        }

        displayRRT();
    }
};

#endif