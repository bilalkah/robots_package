#include "solve_maze/my_mapper.h"

MyMapper::MyMapper(ros::NodeHandle &nh) : nh(nh)
{
    // ROS
    odom_sub = nh.subscribe("/odom", 1, &MyMapper::odomCallback, this);
    laser_sub = nh.subscribe("/scan", 1, &MyMapper::laserCallback, this);

    // Map
    map_width = 960;
    map_height = 960;
    expansion_factor = 20;
    map_origin_x = map_width / 2;
    map_origin_y = map_height / 2;
    map = cv::Mat::zeros(map_height, map_width, CV_8UC1);

}

MyMapper::~MyMapper()
{
}

void MyMapper::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom_pose = *msg;

    // get yaw from quaternion
    double roll, pitch, yaw;
    tf::Quaternion q(
        odom_pose.pose.pose.orientation.x,
        odom_pose.pose.pose.orientation.y,
        odom_pose.pose.pose.orientation.z,
        odom_pose.pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    
    // print x,y,yaw
    std::cout << "x: " << odom_pose.pose.pose.position.x << " y: " << odom_pose.pose.pose.position.y << " yaw: " << current_yaw << std::endl;
}

void MyMapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(laser_mutex);
    laser_scan = *msg;

    cv::Mat baselink = cv::Mat::zeros(map_height, map_width, CV_8UC1);
    cv::circle(baselink, cv::Point(map_origin_x, map_origin_y), 2, cv::Scalar(255), -1);

    for (int i = 0; i < laser_scan.ranges.size(); i++)
    {
        double angle = laser_scan.angle_min + i * laser_scan.angle_increment + current_yaw;
        int x = int (floor(laser_scan.ranges[i] * cos(angle) * expansion_factor));
        int y = int (floor(laser_scan.ranges[i] * sin(angle) * expansion_factor));

        int px = int (floor(odom_pose.pose.pose.position.x * expansion_factor));
        int py = int (floor(odom_pose.pose.pose.position.y * expansion_factor));

        cv::circle(baselink, cv::Point(map_origin_y + y + py, map_origin_x + x + px), 2, cv::Scalar(255), -1);
        cv::circle(map, cv::Point(map_origin_y + y + py,map_origin_x + x + px), 2, cv::Scalar(255), -1);
    }
    cv::Mat map_copy = map.clone();
    // rotate map by 90 degrees counter-clockwise
    cv::transpose(map_copy, map_copy);
    cv::flip(map_copy, map_copy, 1);
    cv::transpose(map_copy, map_copy);
    cv::flip(map_copy, map_copy, 1);
    cv::transpose(map_copy, map_copy);
    cv::flip(map_copy, map_copy, 1);
    cv::imshow("map_rotated", map_copy);
    cv::imshow("baselink", baselink);
    cv::imshow("map", map);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_mapper");
    ros::NodeHandle nh;
    MyMapper my_mapper(nh);
    ros::spin();
    return 0;
}