#include <exploration/exploration.h>

namespace mre
{
    Exploration::Exploration(ros::NodeHandle &nh, TSVector<geometry_msgs::PoseStamped> *exp_point) : nh_(nh), exp_point_(exp_point), min_cluster_(10)
    {
        map_sub_ = nh_.subscribe("/map", 1, &Exploration::mapCallback, this);
        fm_pub_ = nh_.advertise<visualization_msgs::Marker>("/frontier", 1);
    }

    void Exploration::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        nav_msgs::OccupancyGrid map = *msg;
        og_map_ = std::make_unique<OGMap>(map);
        publishFrontier();
        calculateGoals();
    }

    void Exploration::calculateGoals()
    {
        // Connected component labeling
        std::vector<std::vector<geometry_msgs::PoseStamped>> frontier_clusters;
        std::vector<std::vector<bool>> visited(og_map_->height, std::vector<bool>(og_map_->width, false));

        for (int i = 0; i < og_map_->height; i++)
        {
            for (int j = 0; j < og_map_->width; j++)
            {
                auto isFrontier = og_map_->isFrontier(std::make_pair(i, j));
                auto isVisited = visited[i][j];
                if (isFrontier && !isVisited)
                {
                    std::vector<geometry_msgs::PoseStamped> cluster;
                    std::queue<std::pair<int, int>> point_q;
                    point_q.push(std::make_pair(i, j));
                    visited[i][j] = true;
                    int f = 0;
                    while (!point_q.empty() && ros::ok())
                    {
                        f++;
                        std::pair<int, int> p = point_q.front();
                        point_q.pop();
                        geometry_msgs::PoseStamped pose;
                        pose.header.frame_id = "map";
                        pose.pose.position.x = p.second * og_map_->resolution + og_map_->origin.first;
                        pose.pose.position.y = p.first * og_map_->resolution + og_map_->origin.second;
                        cluster.push_back(pose);
                        for (int k = -1; k <= 1; k++)
                        {
                            for (int l = -1; l <= 1; l++)
                            {
                                auto neighbor = std::make_pair<int, int>(p.first + k, p.second + l);
                                auto inBoundary = og_map_->inBoundary(neighbor);

                                if (inBoundary)
                                {
                                    auto isFrontier = og_map_->isFrontier(neighbor);
                                    auto isVisited = visited[neighbor.first][neighbor.second];
                                    if (isFrontier && !isVisited)
                                    {
                                        point_q.push(neighbor);
                                        visited[neighbor.first][neighbor.second] = true;
                                    }
                                }
                            }
                        }
                    }
                    if (cluster.size() > min_cluster_)
                    {
                        frontier_clusters.push_back(cluster);
                    }
                }
            }
        }
        std::vector<geometry_msgs::PoseStamped> goal_vector;
        for (auto &goal : frontier_clusters)
        {
            geometry_msgs::PoseStamped center;
            center.header.frame_id = "map";

            for (auto &pose : goal)
            {
                center.pose.position.x += pose.pose.position.x;
                center.pose.position.y += pose.pose.position.y;
            }
            center.pose.position.x /= goal.size();
            center.pose.position.y /= goal.size();
            goal_vector.emplace_back(center);
        }
        exp_point_->updateVector(goal_vector);
    }

    void Exploration::publishFrontier() const
    {
        visualization_msgs::Marker fm;
        fm.header.frame_id = "map";
        fm.header.stamp = ros::Time::now();
        fm.ns = "frontier";
        fm.id = 0;
        fm.type = visualization_msgs::Marker::POINTS;
        fm.action = visualization_msgs::Marker::ADD;
        fm.pose.orientation.w = 1.0;
        fm.scale.x = 0.1;
        fm.scale.y = 0.1;
        fm.color.a = 1.0;
        fm.color.r = 1.0;
        fm.color.g = 0.0;
        fm.color.b = 0.0;

        for (int i = 0; i < og_map_->height; i++)
        {
            for (int j = 0; j < og_map_->width; j++)
            {
                if (og_map_->frontier[i][j])
                {
                    geometry_msgs::Point p;
                    p.x = j * og_map_->resolution + og_map_->origin.first;
                    p.y = i * og_map_->resolution + og_map_->origin.second;
                    fm.points.push_back(p);
                }
            }
        }
        fm_pub_.publish(fm);
    }
} // namespace mre
