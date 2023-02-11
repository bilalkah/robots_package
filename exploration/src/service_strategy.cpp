#include <exploration/service_strategy.h>

namespace mre
{
    ServiceLogic::ServiceLogic(ros::NodeHandle &nh, const std::string &robot_name, double tolerance) : nh_(nh), robot_name_(robot_name), tolerance_(tolerance), isArrived_(false), state_(RobotState::PASSIVE)
    {
        odom_sub_ = nh_.subscribe("/" + robot_name + "/odom", 1, &ServiceLogic::odomCallback, this);
    }
    bool ServiceLogic::isArrived()
    {
        return isArrived_;
    }
    void ServiceLogic::setGoal(geometry_msgs::PoseStamped goal)
    {
        ROS_INFO_STREAM("Robot " << robot_name_ << " set goal");
        move_base_msgs::MoveBaseGoal move_base_goal;
        goal.pose.orientation = odom_.pose.orientation;
        move_base_goal.target_pose = goal;
        mba_ptr = std::make_unique<MoveBaseClient>("/" + robot_name_ + "/move_base", true);
        while (!mba_ptr->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO_STREAM("Waiting for the move_base action server to come up");
        }
        mba_ptr->sendGoal(move_base_goal);
        state_ = RobotState::ACTIVE;
        serviceClient();
    }
    RobotState ServiceLogic::getState()
    {
        return state_;

    }
    geometry_msgs::PoseStamped ServiceLogic::getPosition()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return odom_;
    }
    void ServiceLogic::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_.pose = msg->pose.pose;
    }
    void ServiceLogic::serviceClient()
    {
        mba_ptr->waitForResult();
        if (mba_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO_STREAM("Robot " << robot_name_ << " arrived");
        }
        ROS_INFO_STREAM("Robot " << robot_name_ << " failed to reach goal");
        state_ = RobotState::PASSIVE;
        isArrived_ = true;
    }
} // namespace mre
