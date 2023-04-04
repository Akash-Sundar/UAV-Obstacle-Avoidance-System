// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"
#include "occupancy_grid.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    // TODO: create a occupancy grid

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::init_occupancy_grid(){

    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));
    if (map_ptr == nullptr){ROS_INFO("No map received");}
    else{
        map = *map_ptr;
        ROS_INFO("Map received");
    }
    ROS_INFO("Initializing occupancy grid for map ...");
    occupancy_grid::inflate_obstacles(map, margin = 0.25);
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // TODO: update your occupancy grid

    map_updated_ = map; 

    float angle_min = scan_msg->angle_min;
    float angle_increment = scan_msg->angle_increment;

    for (int i = 0; i < scan_msg->ranges.size(); i++) {
        float range = scan_msg->ranges.at(i);
        if (range > SCAN_RANGE) {
            continue;
        }
        if (!isnan(range) && !isinf(range)) {
            float angle = angle_min + angle_increment * i;
            if (occupancy_grid::is_xy_occupied(map, pos_in_map.x(), pos_in_map.y())){
                occupancy_grid::inflate_cell(map_updated_, occupancy_grid::xy2ind(map_updated_, pos_in_map.x(), pos_in_map.y()), margin = 0.5, 100);
            }
        }
    }
    // free the cells in which the car occupies (dynamic layer)
    occupancy_grid::inflate_cell(map_updated_, xy2ind(map_updated_, car_pose_.position.x, car_pose_.position.y), 0.25, 0);
    map_update_pub_.publish(map_updated_);

}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<RRT_Node> tree;

    // TODO: fill in the RRT main loop

    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    car_pose_ = pose_msg->pose.pose;
    Node start_node;
    start_node.x = car_pose_.position.x;
    start_node.y = car_pose_.position.y;
    start_node.parent = 0;
    start_node.cost = 0.0;
    start_node.is_root = true;
    //Get Goal
    vector<int> waypoints = get_current_goal();

    std::vector<Node> tree;
    int MAX_ITER = 1000;

    tree.clear();
    tree.push_back(start_node);
    //  RRT main loop
    for (int iter=0; iter<MAX_ITER; iter++){
        vector<double> sampled_point = sample();
        int nearest_ind = nearest(tree, sampled_point);
        Node new_node = steer(tree.at(nearest_ind), sampled_point);

        if (check_collision(tree.at(nearest_ind), new_node) == false){
            vector<int> nodes_near = near(tree, new_node);
            tree.push_back(new_node);

            int min_cost_node_ind = nearest_ind;
            float min_cost = tree.at(nearest_ind).cost + line_cost(tree.at(nearest_ind), new_node);
            for (int i=0; i<nodes_near.size(); i++){
                if(!check_collision(tree.at(nodes_near.at(i)), new_node)){
                    float cost = cost(tree, new_node);
                    float cost = tree.at(nodes_near.at(i)).cost + line_cost(tree.at(nodes_near.at(i)), new_node);
                    if(cost < min_cost) {
                       min_cost_node_ind = nodes_near.at(i);
                       min_cost = cost;
                   }
                }
            }

            tree.back().is_root = false;
            tree.back().cost = min_cost;
            tree.back().parent = min_cost_node_ind;
            tree.at(min_cost_node_ind).children.push_back(tree.size()-1);


            auto p1 = tree.back();
            auto p2 = waypoints.at(current_goal_ind);

            double dist = pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
            dist = sqrt(dist);

            std::vector<Node> nodes_near_goal;

            if (dist < goal_thresh) {
                nodes_near_goal.push_back(p1);
            }
        }

        if (nodes_near_goal.empty()){
        ROS_INFO("Couldn't find a path");
        }

        std::vector<geometry_msgs::Point> path;

        else {
            if (iter > 0.8 * MAX_ITER) {

                vector<Node> path_found = find_path(tree, nodes_near_goal.back());

                for (int i=0; i<path_found.size(); i++){
                    geometry_msgs::Point p;
                    p.x = path_found.at(i).x;
                    p.y = path_found.at(i).y;
                    path.push_back(p)
                }


                path_publisher.publish(path);
                visualize_tree(tree);
                //ROS_INFO("path found");
                break;
            }
        }
    }
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    
    // return sampled_point;

    // std::random_device rd{};
    // std::mt19937 gen{rd()};

    const double mean_x = 5;
    const double mean_y = 5;
    const double std_dev = 1;

    std::normal_distribution<double> norm_dist_x(mean_x, std_dev);
    std::normal_distribution<double> norm_dist_y(mean_y, std_dev);
    double x = norm_dist_x(gen);
    double y = norm_dist_y(gen);

    // sample recursively until one in the free space gets returned
    if (occupancy_grid::is_xy_free(map,x, y)){
        sampled_point.push_back(x);
        sampled_point.push_back(y);
        return sampled_point;
    }
    else{
        return sample();
    }
}

int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method

    double min_dist = 100000.0; // inf
    double dist;
    for (int i=0; i<int(tree.size()); i++){
        auto p1 = tree.at(i);
        auto p2 = sampled_point

        dist = pow(p1.x - p2[0], 2) + pow(p1.y - p2[1], 2);
        // dist = sqrt(dist)

        if (dist<min_dist){
            nearest_node = i;
            min_dist = dist;
        }
    }

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;
    // TODO: fill in this method

    double dist;

    auto p1 = nearest_node;
    auto p2 = sampled_point;

    dist = pow(p1.x - p2[0], 2) + pow(p1.y - p2[1], 2);
    dist = sqrt(dist);

    double grad_x = (p2[0]-p1.x)/dist;
    double grad_y = (p2[1]-p1.y)/dist;

    double max_allowed_steer_distance = 0.25;

    new_node.x = p1.x + min(max_allowed_steer_distance, dist)*grad_x;
    new_node.y = p1.y + min(max_allowed_steer_distance, dist)*grad_y;

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method

    auto p1 = nearest_node;
    auto p2 = new_node;

    int x_diff = abs(ceil(p1.x - p2.x))/map.resolution;
    int y_diff = abs(ceil(p1.y - p2.y))/map.resolution;

    int n = max(x_diff, y_diff);

    for (int i=0, double t = 0.0;i<= n; i++, t += 1.0/n){
        double x = p1.x + t*(p1.x - p2.x);
        double y = p1.y + t*(p1.y - p2.y);

        if (occupancy_grid::is_xy_occupied(map, x, y)){
            collision = true;
            break;
        }
    }

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    auto p1 = latest_added_node;
    auto p2 = [goal_x, goal_y];

    double dist = pow(p1.x - p2[0], 2) + pow(p1.y - p2[1], 2);
    dist = sqrt(dist)

    double goal_threshold = 0.25;
    if (dist < goal_threshold) {
        close_enough = true;
    }

    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    // TODO: fill in this method

    std::vector<RRT_Node> current = node;
    while (!current.is_root){
        found_path.push_back(current);
        current = tree.at(current.parent);
    }
    found_path.push_back(current); 
    reverse(found_path.begin(), found_path.end());

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    std::vector<RRT_Node> current = node;
    while (!current.is_root){
        parent = current.parent;
        cost = RRT::cost(parent) + RRT::line_cost(parent, current);
    }
    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method
    cost = pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2);
    cost = sqrt(cost);

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    double neighborhood_threshold = 0.5;

    for (int i=0; i<int(tree.size()); i++){
        auto p1 = tree.at(i);
        auto p2 = node;

        dist = pow(p1.x - p2[0], 2) + pow(p1.y - p2[1], 2);
        dist = sqrt(dist)

        if (dist<neighborhood_threshold){
            neighborhood.push_back(i)
        }
    }

    return neighborhood;
}

