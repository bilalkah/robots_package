#pragma once

// ROS Header
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// STL Header
#include <vector>
#include <queue>
#include <array>

namespace patrol
{

    // Represent occupancy_grid map as graph nodes
    class MapToGraph
    {
    public:
        MapToGraph(ros::NodeHandle &nh, const size_t &num_robot);
        ~MapToGraph();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber map_sub_;
        ros::Publisher graph_pub_, segmented_map_pub_, route_pub_;
        std::vector<ros::Publisher> path_pub_;

        size_t num_robot_;
        int loop_;

        struct MapData
        {
            size_t height_, width_;
            double resolution_;
            std::array<double, 2> origin_;
            std::vector<std::vector<size_t>> map_;
        };
        std::unique_ptr<MapData> map_data_{};

        size_t min_area_, max_area_;
        std::unique_ptr<std::vector<std::vector<size_t>>> segmented_map_;
        std::vector<std::tuple<size_t, std::array<double, 2>>> regions;
        size_t region_num;

        std::unique_ptr<std::vector<std::vector<bool>>> graph_;

        // Callbacks
        void mapCallback(const nav_msgs::OccupancyGrid &map);
        void makeSegmentation();
        void makeGraph();
        void makeRoute();

        // Visualization
        visualization_msgs::Marker node_, connection_;
    };
} // namespace
