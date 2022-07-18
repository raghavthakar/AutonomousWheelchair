#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "std_msgs/Float64.h"
#include <string>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <wheelchair_control/MoveDistanceAction.h>
#include <wheelchair_control/MoveDistanceGoal.h>
#include <wheelchair_control/MoveDistanceResult.h>
#include <wheelchair_control/MoveDistanceFeedback.h>

// linear velocity of the robot
#define BASE_LINEAR_VEL 0.25

const double KP=0.5;
const double DT=0.65;
const double target_reached_acceptable_distance = 0.1;

class Mover
{
    ros::NodeHandle node_handle;
    actionlib::SimpleActionServer<wheelchair_control::MoveDistanceAction> server;
    nav_msgs::OdometryConstPtr current_odom;
    std::string odom_topic;
    std::string twist_topic;
    geometry_msgs::Twist twist_message;
    ros::Publisher twist_publisher;
    ros::Subscriber odom_subscriber;

    void odom_callback(nav_msgs::OdometryConstPtr msg)
    {
        // ROS_INFO("In the callback");
        current_odom=msg;
    }

    std::vector<double> getOrientation()
    {
        double roll, pitch, yaw;
        std::vector<double> angles;
        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(current_odom->pose.pose.orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        angles.push_back(roll);
        angles.push_back(pitch);
        angles.push_back(yaw);

        return angles;
    }

    double getCurrentYaw()
    {
        double roll, pitch, yaw;

        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(current_odom->pose.pose.orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        return yaw;
    }

    double distanceFrom(double target_x, double target_y)
    {
        return sqrt(pow(target_y-current_odom->pose.pose.position.y, 2)
                +pow(target_x-current_odom->pose.pose.position.x, 2));
    }

    double getAngleToTarget(double target_y, double current_y,
                          double target_x, double current_x)
    {
        //Find the angle to rotate to face target
        double angle_to_target = atan((target_y-current_y)/
                                      (target_x-current_x));

        if(target_x-current_x<=0)
          if(target_y-current_y>=0)
            angle_to_target+=3.14;
          else
            angle_to_target-=3.14;

        if(angle_to_target>3.14)
            angle_to_target-=3.14;
        else if(angle_to_target<-3.14)
            angle_to_target+=3.14;

        return angle_to_target;
    }

    void move(const wheelchair_control::MoveDistanceGoalConstPtr& goal)
    {
        // tracks the success of the action
        bool action_success=true;

        // to store the feedback to be returned
        wheelchair_control::MoveDistanceFeedback feedback;
        // to store the goal to be returned
        wheelchair_control::MoveDistanceResult result;

        double angle_to_target=getAngleToTarget(goal->target.y, current_odom->
                                      pose.pose.position.y, goal->target.x,
                                      current_odom->pose.pose.position.x);

        ROS_INFO("TARGET X: %f Y: %f", goal->target.x, goal->target.y);

        //linear velocity will always be this
        twist_message.linear.x=BASE_LINEAR_VEL;

        //Move the robot forwards till we reach the target
        while(fabs(distanceFrom(goal->target.x, goal->target.y))>target_reached_acceptable_distance)
        {
            // handle preempt request
            if(server.isPreemptRequested() || !ros::ok())
            {
                ROS_WARN("Preempt requested");
                server.setPreempted();
                action_success=false;
            }

            double yaw_error= getAngleToTarget(goal->target.y, current_odom->
                                          pose.pose.position.y, goal->target.x,
                                          current_odom->pose.pose.position.x) - getCurrentYaw();

            // For angular controller
            double p_effort=yaw_error*KP;

            // Set the angular yaw
            twist_message.angular.z=p_effort;

            twist_publisher.publish(twist_message);
            // ROS_INFO("DISTANCE TO TARGET: %f", distanceFrom(goal->target.x, goal->target.y));
            // ROS_INFO("YAW ERROR: %f", yaw_error);
            // ROS_INFO("P EFFORT: %f", p_effort);

            // STORE THe feedback
            feedback.distance_left=(double)fabs(distanceFrom(goal->target.x, goal->target.y));
            // publish the feedback (distance from target)
            server.publishFeedback(feedback);
            ros::spinOnce();
        }

        // flow is here if either success or failure of action
        // if success
        if(action_success)
        {
            ROS_INFO("Action succeeded!");
            result.current_pose=current_odom->pose.pose;
            server.setSucceeded(result);
        }

        //Stop the robot
        twist_message.angular.z=0;
        twist_message.linear.x=0;
        twist_publisher.publish(twist_message);

    }

public:
    // consructor with intitializer list
    Mover(char** argv):server(node_handle, "move_distance", boost::bind(&Mover::move, this, _1), false)
    {
        ROS_INFO_STREAM("in the constructor");
        // setting up the variables that will be used throughout
        odom_topic=odom_topic.append("odom");

        twist_topic=twist_topic.append("cmd_vel");

        twist_publisher = node_handle.advertise
                          <geometry_msgs::Twist>(twist_topic, 10);

        odom_subscriber = node_handle.subscribe(odom_topic, 15, &Mover::odom_callback, this);

        current_odom=ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic);

        server.start();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_distance_server");
    Mover mover(argv);
    ros::spin();
    return 0;
}
