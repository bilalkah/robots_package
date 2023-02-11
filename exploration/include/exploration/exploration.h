#ifndef EXPLORATION_EXPLORATION_H
#define EXPLORATION_EXPLORATION_H

#include <exploration/utils/ts_vector.h>
#include <exploration/utils/og_map.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <queue>
#include <numeric>
#include <string>

namespace mre
{
    class Exploration
    {
    public:
        Exploration(ros::NodeHandle &nh, std::shared_ptr<TSVector<geometry_msgs::PoseStamped>>exp_point, int min_cluster);

    private:
        ros::NodeHandle &nh_;
        ros::Subscriber map_sub_;
        ros::Publisher fm_pub_;

        std::unique_ptr<OGMap> og_map_;
        std::shared_ptr<TSVector<geometry_msgs::PoseStamped>> exp_point_;
        int min_cluster_;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void calculateGoals();
        void publishFrontier() const;
    };
} // namespace mre

#endif // EXPLORATION_EXPLORATION_H