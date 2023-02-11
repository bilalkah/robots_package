#ifndef EXPLORATION_SERVICE_STRATEGY_H
#define EXPLORATION_SERVICE_STRATEGY_H

#include <exploration/robot_control_strategy.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

#include <string>
#include <memory>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace mre
{
    class ServiceLogic : public RobotControlLogic
    {
    public:
        explicit ServiceLogic(ros::NodeHandle &nh, const std::string &robot_name, double tolerance);
        
        void setGoal(geometry_msgs::PoseStamped goal) override;

        bool isArrived() override;
        RobotState getState() override;
        geometry_msgs::PoseStamped getPosition() override;

    private:
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;

        geometry_msgs::PoseStamped odom_;
        std::unique_ptr<MoveBaseClient> mba_ptr;

        std::string robot_name_;
        double tolerance_;
        RobotState state_;
        bool isArrived_;

        std::mutex mutex_;
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void serviceClient();
    };
} // namespace mre

#endif // EXPLORATION_SERVICE_STRATEGY_H