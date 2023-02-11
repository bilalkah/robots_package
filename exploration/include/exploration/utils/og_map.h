#ifndef EXPLORATION_UTILS_OG_MAP_H
#define EXPLORATION_UTILS_OG_MAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <numeric>
namespace mre
{

    struct OGMap
    {
        OGMap(nav_msgs::OccupancyGrid map);
        void calculateFrontiers();
        bool unkownNeighbor(std::pair<int, int> index);
        bool isFree(std::pair<int, int> index);
        bool isFrontier(std::pair<int, int> index);
        bool isUnknown(std::pair<int, int> index);
        bool inBoundary(std::pair<int, int> index);

        std::vector<std::vector<int>> map_data;
        std::vector<std::vector<bool>> frontier;
        size_t height, width;
        double resolution;
        std::pair<double, double> origin;
    };

} // namespace mre
#endif // EXPLORATION_UTILS_OG_MAP_H