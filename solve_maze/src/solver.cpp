#include "solve_maze/solver.h"

solve_maze::Solver::Solver(ros::NodeHandle &nh) : nh(nh)
{
    // ROS
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    laser_sub = nh.subscribe("/scan", 1, &Solver::laserCallback, this);
    odom_sub = nh.subscribe("/odom", 1, &Solver::odomCallback, this);

    linear_speed = 0.3;
    angular_speed = 0.5;
    min_range = 0.6;
    empth_path_range = 1.2;

    magnitude = 10;
    front_l = 20;
    front_r = 340;

    left_min = 90 - magnitude;
    left_max = 90 + magnitude;
    right_min = 270 - magnitude;
    right_max = 270 + magnitude;
    back_min = 180 - magnitude;
    back_max = 180 + magnitude;

    ROS_INFO("Solver node is ready.");
}

solve_maze::Solver::~Solver()
{
}

void solve_maze::Solver::run()
{
    ros::Rate rate(5);
    // sleep and wait for the first laser scan
    for (int i = 0; i < 10; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }
    while (ros::ok())
    {
        // ROS_INFO("Running");
        bool turn_left, dont_turn = false;
        {
            std::lock_guard<std::mutex> lock(laser_mutex);

            ROS_INFO_STREAM(left_clear << " " << right_clear << " " << back_clear);

            // generate random number between 0 and 1
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);

            // if random number is less than 0.3, don't turn
            // if random number is between 0.3 and 0.6, turn left
            // if random number is greater than 0.6, turn right
            if (dis(gen) < 0.5)
            {
                dont_turn = true;
            }
            else if (dis(gen) < 0.75)
            {
                turn_left = true;
            }
            else
            {
                turn_left = false;
            }

            if (!dont_turn)
            {
                // if left is clear turn left
                if (turn_left && left_clear)
                {
                    ROS_INFO("Turning left");
                    target_yaw = current_yaw + M_PI / 2;
                    if (target_yaw > M_PI)
                        target_yaw -= 2 * M_PI;
                    else if (target_yaw < -M_PI)
                        target_yaw += 2 * M_PI;

                    // round to 90, 180, 270, 360
                    target_yaw = round(target_yaw / (M_PI / 2)) * (M_PI / 2);
                }
                // if right is clear turn right
                else if(!turn_left && right_clear)
                {
                    ROS_INFO("Turning right");
                    target_yaw = current_yaw - M_PI / 2;
                    if (target_yaw > M_PI)
                        target_yaw -= 2 * M_PI;
                    else if (target_yaw < -M_PI)
                        target_yaw += 2 * M_PI;
                    // round to 90, 180, 270, 360
                    target_yaw = round(target_yaw / (M_PI / 2)) * (M_PI / 2);
                }
            }
            else
            {
                ROS_INFO("Not turning");
            }
        }
        bool turned = false;
        if (!dont_turn && (left_clear || right_clear))
        {
            turning = true;
            // publish the angular velocity to turn until the target yaw is reached
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = angular_speed;
            if (!turn_left)
                cmd_vel.angular.z *= -1;
            while (ros::ok() && fabs(target_yaw - current_yaw) > 0.1)
            {
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                rate.sleep();
            }
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
            turned = true;
        }
        turning = false;
        nav_msgs::Odometry odom;
        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            odom = odom_pose;
        }
        if (turned)
        {
            // wait odom difference bigger than 1.5 meters
            double x = odom.pose.pose.position.x;
            double y = odom.pose.pose.position.y;
            while (ros::ok() && (fabs(x - odom_pose.pose.pose.position.x) < 1.5 && fabs(y - odom_pose.pose.pose.position.y) < 1.5))
            {
                ros::spinOnce();
                rate.sleep();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void solve_maze::Solver::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    std::lock_guard<std::mutex> lock(laser_mutex);
    laser_scan = *msg;

    // check left of the robot
    left_clear = true;
    for (int i = left_min; i <= left_max; i++)
    {
        if (laser_scan.ranges[i] < empth_path_range)
        {
            left_clear = false;
            break;
        }
    }

    right_clear = true;
    for (int i = right_min; i <= right_max; i++)
    {
        if (laser_scan.ranges[i] < empth_path_range)
        {
            right_clear = false;
            break;
        }
    }
    back_clear = true;
    for (int i = back_min; i <= back_max; i++)
    {
        if (laser_scan.ranges[i] < empth_path_range)
        {
            back_clear = false;
            break;
        }
    }
    ranges.clear();
    int num_readings = laser_scan.ranges.size();
    for (int i = front_l; i >= 0; i--)
    {
        ranges.push_back(laser_scan.ranges[i]);
    }
    for (int i = num_readings - 1; i >= front_r; i--)
    {
        ranges.push_back(laser_scan.ranges[i]);
    }
    if (turning)
    {
        return;
    }
    bool is_free = true;
    for (int i = 0; i < ranges.size(); i++)
    {
        if (ranges[i] < min_range)
        {
            is_free = false;
            break;
        }
    }

    // ROS_INFO_STREAM("is_free: " << is_free);
    //  if free, move forward
    if (is_free)
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel);
    }
    //  if not free
    else
    {
        // check where the obstacle is
        bool is_left = false;
        bool is_right = false;
        for (int i = 0; i < ranges.size(); i++)
        {
            if (ranges[i] < min_range)
            {
                if (i < ranges.size() / 2)
                {
                    is_left = true;
                }
                else
                {
                    is_right = true;
                }
            }
        }
        // if obstacle is on the left, turn right
        // ROS_INFO_STREAM("is_left: " << is_left << " is_right: " << is_right);
        if (is_left)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -angular_speed;
            cmd_vel_pub.publish(cmd_vel);
        }
        // if obstacle is on the right, turn left
        else if (is_right)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = angular_speed;
            cmd_vel_pub.publish(cmd_vel);
        }
    }
}

void solve_maze::Solver::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom_pose = *msg;
    // get current yaw
    double roll, pitch, yaw;
    tf::Quaternion q(
        odom_pose.pose.pose.orientation.x,
        odom_pose.pose.pose.orientation.y,
        odom_pose.pose.pose.orientation.z,
        odom_pose.pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    // ROS_INFO_STREAM("current_yaw: " << current_yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solver");
    ros::NodeHandle nh;
    solve_maze::Solver solver(nh);
    solver.run();
    return 0;
}
