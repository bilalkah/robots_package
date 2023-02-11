#include <exploration/topic_strategy.h>

namespace mre
{
    TopicLogic::TopicLogic(ros::NodeHandle &nh, const std::string robot_name, double tolerance) : nh_(nh), robot_name_(robot_name), tolerance_(tolerance), state_(RobotState::PASSIVE)
    {
        odom_sub_ = nh_.subscribe("/" + robot_name + "/odom", 1, &TopicLogic::odomCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + robot_name + "/move_base_simple/goal", 1);
        ROS_INFO_STREAM("TopicLogic is up for robot " << robot_name << ".");
    }
    bool TopicLogic::isArrived()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto isRobotStuck = std::hypot(odom_.pose.position.x - prev_odom_.pose.position.x, odom_.pose.position.y - prev_odom_.pose.position.y) >= tolerance_;
        double distance = std::hypot(odom_.pose.position.x - goal_.pose.position.x, odom_.pose.position.y - goal_.pose.position.y);
        if (distance <= tolerance_ && !isRobotStuck)
        {
            ROS_INFO_STREAM("Robot " << robot_name_ << " arrived");
            state_ = RobotState::PASSIVE;
            return true;
        }
        return false;
    }
    void TopicLogic::setGoal(geometry_msgs::PoseStamped goal)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ROS_INFO_STREAM("Robot " << robot_name_ << " set goal");
        goal.pose.orientation = odom_.pose.orientation;
        goal_ = goal;
        goal_pub_.publish(goal_);
        state_ = RobotState::ACTIVE;
    }
    RobotState TopicLogic::getState()
    {
        return state_;
    }

    geometry_msgs::PoseStamped TopicLogic::getPosition()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return odom_;
    }

    void TopicLogic::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        static auto time = ros::Time::now();
        if(ros::Time::now() - time >= ros::Duration(1.5))
        {
            prev_odom_ = odom_;   
        }
        odom_.pose = msg->pose.pose;
    }
} // namespace mre
