#include <exploration/robot.h>

namespace mre
{
    Robot::Robot(RobotControlLogic *rcl_ptr, TSVector<geometry_msgs::PoseStamped> *exp_point) : exp_point_(exp_point)
    {
        rcl_ptr_.reset(rcl_ptr);

        std::thread t(&Robot::run, this);
        t.detach();
    }

    Robot::~Robot()
    {
        // delete rcl_ptr_;
    }

    void Robot::run()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            if (rcl_ptr_->getState() == RobotState::PASSIVE)
            {
                tryGetGoal();
            }
            else
            {
                rcl_ptr_->isArrived();
            }
            rate.sleep();
        }
    }

    void Robot::tryGetGoal()
    {
        // ROS_INFO("Try Get Goal");
        if (!exp_point_->empty())
        {
            ROS_INFO_STREAM("Robot is active");
            geometry_msgs::PoseStamped goal = exp_point_->closest(rcl_ptr_->getPosition());
            ROS_INFO_STREAM("Robot is going to " << goal.pose.position.x << ", " << goal.pose.position.y);
            rcl_ptr_->setGoal(goal);
        }
    }
} // namespace mre
