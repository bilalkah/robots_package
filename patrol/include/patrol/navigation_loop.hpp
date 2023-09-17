#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <queue>
#include <string>
#include <mutex>
#include <thread>

namespace patrol
{
    class Robot
    {
    public:
        Robot(ros::NodeHandle &nh, std::string robot_id);
        ~Robot();

        void run();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_, path_sub_, scan_sub_;
        ros::Publisher goal_pub_;
        std::string odom_topic_, path_topic_, robot_id_, goal_topic_, scan_topic_;
        geometry_msgs::PoseStamped current_pose_;
        nav_msgs::Path path_;
        bool path_received_;
        bool active_;
        double tolerance_;
        double scan_area_;
        std::queue<geometry_msgs::PoseStamped> path_queue_;

        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void pathCallback(const nav_msgs::Path::ConstPtr &msg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        bool isReached(geometry_msgs::PoseStamped pose);
    };
} // namespace patrol
