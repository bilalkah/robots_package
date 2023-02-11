#ifndef EXPLORATION_TOPIC_STRATEGY_H
#define EXPLORATION_TOPIC_STRATEGY_H

#include <exploration/robot_control_strategy.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <string>

namespace mre
{
    class TopicLogic : public RobotControlLogic
    {
    public:
        explicit TopicLogic(ros::NodeHandle &nh, const std::string robot_name, double tolerance);
        void setGoal(geometry_msgs::PoseStamped goal) override;

        bool isArrived() override;
        RobotState getState() override;
        geometry_msgs::PoseStamped getPosition() override;

    private:
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;
        ros::Publisher goal_pub_;

        geometry_msgs::PoseStamped goal_, odom_, prev_odom_;
        double tolerance_;
        std::string robot_name_;
        RobotState state_;

        std::mutex mutex_;

        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    };
} // namespace mre

#endif // EXPLORATION_TOPIC_STRATEGY_H