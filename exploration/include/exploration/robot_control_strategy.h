#ifndef EXPLORATION_ROBOT_CONTROL_STRATEGY_H
#define EXPLORATION_ROBOT_CONTROL_STRATEGY_H

#include <geometry_msgs/PoseStamped.h>

// Multi Robot Exploration
namespace mre
{
    enum class RobotState
    {
        PASSIVE,
        ACTIVE,
    };

    class RobotControlLogic
    {
    public:
        virtual void setGoal(geometry_msgs::PoseStamped goal) = 0;
        virtual bool isArrived() = 0;

        virtual RobotState getState() = 0;
        virtual geometry_msgs::PoseStamped getPosition() = 0;
    };
} // namespace mre

#endif // EXPLORATION_ROBOT_CONTROL_STRATEGY_H