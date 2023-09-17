#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>
#include <mutex>

class MyMapper
{
public:
    MyMapper(ros::NodeHandle &nh);
    ~MyMapper();
    void run();

private:
    // ROS
    ros::NodeHandle nh;

    // ROS Callbacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    // ROS Subscribers
    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;

    // ROS Publishers
    ros::Publisher map_pub;

    // ROS Messages
    nav_msgs::Odometry odom_pose;
    sensor_msgs::LaserScan laser_scan;

    // Map
    cv::Mat map;
    int map_width, map_height;
    double map_resolution;
    double map_origin_x, map_origin_y;
    
    // Mutex
    std::mutex odom_mutex, laser_mutex;

    double current_yaw;
    double expansion_factor;

    
};