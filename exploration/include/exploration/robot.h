#ifndef EXPLORATION_ROBOT_H
#define EXPLORATION_ROBOT_H

// #include <exploration/robot_control_strategy.h>
#include <exploration/topic_strategy.h>
#include <exploration/service_strategy.h>
#include <exploration/utils/ts_vector.h>
#include <exploration/exploration.h>

#include <ros/ros.h>

#include <thread>
#include <memory>
#include <string>
#include <iostream>

namespace mre
{
    class Robot
    {
    public:
        Robot(RobotControlLogic *rcl_ptr, std::shared_ptr<TSVector<geometry_msgs::PoseStamped>> exp_point);
        ~Robot();
    private:
        // RobotControlLogic* rcl_ptr_;
        std::unique_ptr<RobotControlLogic> rcl_ptr_;
        std::shared_ptr<TSVector<geometry_msgs::PoseStamped>> exp_point_;

        void run();
        void tryGetGoal();
    };
} // namespace mre

#endif // EXPLORATION_ROBOT_H