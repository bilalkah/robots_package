#ifndef _SOLVER_H_
#define _SOLVER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <iostream>
#include <mutex>
#include <thread>
#include <random>
#include <chrono>

namespace solve_maze
{
    class Solver
    {
    public:
        Solver(ros::NodeHandle &nh);
        ~Solver();
        void run();

    private:
        // ROS
        ros::NodeHandle nh;
        ros::Publisher cmd_vel_pub;
        ros::Publisher odom_pub;
        ros::Subscriber laser_sub;
        ros::Subscriber odom_sub;

        nav_msgs::Odometry odom_pose;
        sensor_msgs::LaserScan laser_scan;

        // ROS Callbacks
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

        // speed
        double linear_speed;
        double angular_speed;
        double min_range, empth_path_range;
        double current_yaw, target_yaw;
        std::vector<double> ranges;

        // magnitude
        int magnitude;
        // front
        int front_l, front_r;
        // left
        int left_min, left_max;
        // right
        int right_min, right_max;
        // back
        int back_min, back_max;

        bool left_clear, right_clear, back_clear, turning;

        std::mutex laser_mutex,odom_mutex;
    };
}

#endif