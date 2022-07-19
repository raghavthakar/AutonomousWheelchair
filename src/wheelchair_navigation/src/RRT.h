#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <list>
#include <random>
#include <cmath>
#include <ctime>
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
    double cost; //cost to travel from root to this node

public:
    Node(){ 
        this->cost = 0;
    }

    Node(Point coords)
    {
        this->coords = coords;
        this->cost = 0;
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

    void setCoords(Point coords)
    {
        this->coords = coords;
    }

    void setCost(double cost)
    {
        this->cost = cost;
    }

    Node* getParent()
    {
        return this->parent;
    }

    Point getCoords()
    {
        return coords;
    }

    double getCost()
    {
        return this->cost;
    }

    void display()
    {
        ROS_INFO("X:%d Y:%d Cost:%g", coords.x, coords.y, cost);
    }

    void addChild(Node* child_node)
    {
        this->children.push_back(child_node);
    }

    void setParent(Node* parent_node)
    {
        this->parent = parent_node;
    }

    // set the pointer to parent and update cost of node
    void setParentAugmentCost(Node* parent_node, double distance)
    {
        this->parent = parent_node;
        this->cost = this->parent->getCost() + distance;
    }

    std::list<Node*> getChildren()
    {
        return this->children;
    }

    void removeChild(Node* child_node)
    {
        try
        {
            this->children.remove(child_node);
        }
        catch(int err)
        {
            ;
        }
    }
};

class RRTHandler
{
    nav_msgs::OccupancyGrid map; //store the map object
    unsigned int map_dim; //store the dimension of the map for reference
    Point start_point;
    Point target_point;
    unsigned int step_length; //the maximum distance (in pixels) between connected nodes
    unsigned int search_radius; //the readius within which qnew searches for improved connections
    std::list<Node*> all_nodes; //keep track of all nodes to easily find them 

    float distanceBetween(Node* node1, Node* node2)
    {
        return(sqrt((node1->getCoords().x-node2->getCoords().x)*(node1->getCoords().x-node2->getCoords().x)
                     + (node1->getCoords().y-node2->getCoords().y)*(node1->getCoords().y-node2->getCoords().y)));
    }

    // generates a random configuration within the workspace and returns it as a Point
    Node randomConfig()
    {
        sleep(0.1);
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
            // ROS_INFO("Min dist: %f", min_dist);
        }
        return q_near;
    }

    // generates a new configuration in the directio of q_rand from q_near but within step_length
    Node newConfiguration(Node* q_rand, Node* q_near)
    {
        Node q_new;
        float nodes_distance = distanceBetween(q_rand, q_near);
        if(nodes_distance < (float) this->step_length)
        {
            q_new.setCoords(q_rand->getCoords());
        }
        else
        {
            // Compute the slope between q_rand and q_new
            double delta_x = (double) q_rand->getCoords().x-(double) q_near->getCoords().x;
            double delta_y = (double) q_rand->getCoords().y-(double) q_near->getCoords().y;
            double theta = atan2(delta_y, delta_x);

            // compute the new point q_new
            int new_x = q_near->getCoords().x + this->step_length * cos(theta);
            int new_y = q_near->getCoords().y + this->step_length * sin(theta);
            ROS_INFO("THETA: %f", theta);
            ROS_INFO("Q_NEAR coords: %d %d", q_near->getCoords().x, q_near->getCoords().y);
            ROS_INFO("New coords: %d %d", new_x, new_y);

            q_new.setCoords(new_x, new_y);
        }

        return q_new;
    }

    //Returns a vector of all neighbours of q_new within the search_radius
    std::vector<Node*> searchNeighbours(Node* q_new, Node* q_near)
    {
        std::vector<Node*> q_neighbours;
        for(Node* node:this->all_nodes)
        {
            if(distanceBetween(node, q_new) <= this->search_radius && node != q_near)
                q_neighbours.push_back(node);
        }

        return q_neighbours;
    }

    // return the neigbour connecting with whom gives best connection cost-wise
    Node* pickBestNeighbour(std::vector<Node*> q_neighbours, Node* q_near, Node* q_new)
    {
        if(q_neighbours.empty()) // if there are no neighbours then simply return q_near
            return q_near;
        
        Node* q_best = q_near; //by default connect with q_near
        for (Node* neighbour:q_neighbours)
        {
            // if connecting to a neighbour is better than q_best, then make q_best the neighbour
            if(neighbour->getCost() + distanceBetween(neighbour, q_new) < q_best->getCost() + distanceBetween(q_best, q_new))
                q_best = neighbour;
        }

        return q_best;
    }

    // checks if a node is occupied or not
    bool isOccupied(Node* node)
    {
        ROS_INFO("Occupation: %d", (int) this->map.data[node->getCoords().y*this->map_dim+node->getCoords().x]);
        
        if ((int) this->map.data[node->getCoords().y*this->map_dim+node->getCoords().x] == 0)
            return false;
        else
            return true;
    }

    // checks if there is an obstacle between q_near and q_new
    bool obstacleBetween(Node* q_near, Node* q_new)
    {
        int limit_x = q_new->getCoords().x - q_near->getCoords().x;
        int limit_y = q_new->getCoords().y - q_near->getCoords().y;

        // iterate the whole triangle from q_near to q_new and return true if any element is an obstacle
        for(int iter_x=q_near->getCoords().x; iter_x!=q_new->getCoords().x; iter_x+=limit_x/abs(limit_x))
        {
            for(int iter_y=q_near->getCoords().y; iter_y!=q_new->getCoords().y; iter_y+=limit_y/abs(limit_y))
            {
                if(this->map.data[iter_y*this->map_dim+iter_x] == 100)
                    return true;
            }
        }

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

    // save the Tree as a CSV file
    void saveTree()
    {
        std::ofstream csv_file("/home/raghav/OntarioTech/AutonomousWheelchair/src/wheelchair_navigation/src/sample_maps/tree_nodes.csv", std::ios::out | std::ios::binary);
        for(auto node:this->all_nodes)
        {
            csv_file << node->getCoords().x << "," <<node->getCoords().y ;
            for(auto child_node:node->getChildren())
            {
                csv_file << ":" << child_node->getCoords().x << "," <<child_node->getCoords().y;
            }
            csv_file << "\n";
        }
        csv_file.close();
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

        // set the default search radius value
        this->search_radius = 20;
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

    void setSearchRadius(unsigned int search_radius)
    {
        this->search_radius = search_radius;
    }

    void RRT(unsigned int num_iterations)
    {
        unsigned int iteration_count = 0; //keeps track of the number of iterations that have taken place
        
        Node* q_root = new Node(this->start_point); //starting node at the start point
        this->all_nodes.push_back(q_root); //add the starting node to list of all nodes

        while(iteration_count < num_iterations)
        {
            std::cout<<"\n";
            Node* q_rand = new Node(randomConfig()); //random configuration generated on the map
            ROS_INFO("Random config: ");
            q_rand->display();

            Node* q_near = closestToRandomConfig(q_rand); //search through all nodes to find the one closest to random configuration
            ROS_INFO("Near node: ");
            q_near->display();

            Node* q_new = new Node(newConfiguration(q_rand, q_near)); //generate a new configuration along q_rand but within step length

            std::vector<Node*> q_neighbours = searchNeighbours(q_new, q_near); //search and return the neigbours of q_new withing the search radius (except q_near)
            ROS_INFO("Neighbours: ");
            for(auto i:q_neighbours)
                i->display();

            Node* q_best = pickBestNeighbour(q_neighbours, q_near, q_new); //find the neighbour connecting with whom will result in lowest cost
            ROS_INFO("Best neighbour: ");
            q_best->display();

            if(isOccupied(q_new) || obstacleBetween(q_best, q_new))
            {
                continue;
            }

            q_near->addChild(q_new); //make new node the child node of nearest node
            q_new->setParentAugmentCost(q_best, distanceBetween(q_new, q_best)); //make nearest node the parent of new node and and add distance to the cost of q_new
            this->all_nodes.push_back(q_new); //add the new node to list of all nodes
            
            ROS_INFO("New Node: ");
            q_new->display();

            q_neighbours.push_back(q_near); //add q_near to set of all neighbours of q_new
            for(Node* neighbour:q_neighbours) //iterate through all neighbours to check for possible rewiring
            {
                if(neighbour->getCost() > q_new->getCost() + distanceBetween(neighbour, q_new)) //if reaching the neighbour via q_new is cheaper
                {
                    neighbour->getParent()->removeChild(neighbour); //remove neighbour as a child from its parent
                    neighbour->setParent(q_new); //make q_new the parent
                    neighbour->setCost(q_new->getCost() + distanceBetween(neighbour, q_new)); //set the new cost to reach neighbour
                    q_new->addChild(neighbour); //add the neighbour to q_new's list of children

                }
            }

            iteration_count++;
        }

        saveTree();
        displayRRT();
    }
};

#endif