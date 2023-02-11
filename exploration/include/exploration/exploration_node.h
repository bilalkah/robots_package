#ifndef EXPLORATION_EXPLORATION_NODE_H
#define EXPLROATION_EXPLORATION_NODE_H

#include <exploration/robot.h>
#include <exploration/topic_strategy.h>
#include <exploration/service_strategy.h>
#include <exploration/exploration.h>

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <thread>
#include <memory>

namespace mre
{

    class ExplorationNode
    {
    public:
        ExplorationNode(ros::NodeHandle &nh);
        ~ExplorationNode();
    private:
        ros::NodeHandle nh_;
        std::unique_ptr<Exploration> exp_ptr_;
        std::vector<std::unique_ptr<Robot>> v_robot_ptr_;
        std::shared_ptr<TSVector<geometry_msgs::PoseStamped>> exploration_points_ptr_;
    };

} // namespace mre

#endif // EXPLORATION_EXPLORATION_NODE_H