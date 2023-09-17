#include <patrol/navigation_loop.hpp>

patrol::Robot::Robot(ros::NodeHandle &nh, std::string robot_id)
    : nh_(nh), robot_id_(robot_id)
{
    odom_topic_ = "/tb3_" + robot_id_ + "/odom";
    path_topic_ = "/tb3_" + robot_id_ + "/path";
    goal_topic_ = "/tb3_" + robot_id_ + "/move_base_simple/goal";
    scan_topic_ = "/tb3_" + robot_id_ + "/scan";

    ROS_INFO_STREAM("odom_topic: " << odom_topic_);
    ROS_INFO_STREAM("path_topic: " << path_topic_);
    ROS_INFO_STREAM("goal_topic: " << goal_topic_);
    ROS_INFO_STREAM("scan_topic: " << scan_topic_);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &Robot::odomCallback, this);
    path_sub_ = nh_.subscribe(path_topic_, 1, &Robot::pathCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &Robot::scanCallback, this);
    path_received_ = false;
    active_ = false;
    tolerance_ = 0.75;

    ROS_INFO_STREAM("robot_id: " << robot_id_);
}

patrol::Robot::~Robot()
{
}

void patrol::Robot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose_.pose = msg->pose.pose;
}

void patrol::Robot::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    path_ = *msg;
    path_received_ = true;
}

void patrol::Robot::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // calculate the scan area as the area of the circle

    sensor_msgs::LaserScan scan = *msg;
}

bool patrol::Robot::isReached(geometry_msgs::PoseStamped pose)
{
    double x = current_pose_.pose.position.x;
    double y = current_pose_.pose.position.y;
    double x_goal = pose.pose.position.x;
    double y_goal = pose.pose.position.y;
    double distance = sqrt(pow(x - x_goal, 2) + pow(y - y_goal, 2));
    if (distance < tolerance_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void patrol::Robot::run()
{
    geometry_msgs::PoseStamped goal;
    while (ros::ok())
    {
        if (path_received_)
        {
            // erase the old path
            while (!path_queue_.empty())
            {
                path_queue_.pop();
            }
            for (auto pose : path_.poses)
            {
                path_queue_.push(pose);
            }
            path_received_ = false;
        }
        if (!path_queue_.empty())
        {
            if (!active_)
            {
                goal = path_queue_.front();
                path_queue_.pop();
                path_queue_.push(goal);
                active_ = true;
            }

            if (isReached(goal))
            {
                active_ = false;
            }
            else
            {
                goal.header.stamp = ros::Time::now();
                goal.header.frame_id = "map";
                goal.pose.orientation.w = 1.0;
                goal_pub_.publish(goal);
                ROS_INFO_STREAM("robot_id: " << robot_id_ << " goal: " << goal.pose.position.x << ", " << goal.pose.position.y);
            }
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_loop");
    ros::NodeHandle nh;

    int robot_num = 3;
    std::vector<std::thread> threads;
    std::vector<std::shared_ptr<patrol::Robot>> robots;
    for (int i = 0; i < robot_num; i++)
    {
        std::string robot_id = std::to_string(i);
        std::shared_ptr<patrol::Robot> robot(new patrol::Robot(nh, robot_id));
        robots.push_back(robot);
        threads.push_back(std::thread(&patrol::Robot::run, robot));

        // detach the thread
        threads[i].detach();
    }

    ros::spin();

    return 0;
}