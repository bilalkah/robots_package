#include <patrol/map_to_graph.hpp>

patrol::MapToGraph::MapToGraph(ros::NodeHandle &nh, const size_t &num_robot)
{
    nh_ = nh;

    map_sub_ = nh_.subscribe("/map", 1, &MapToGraph::mapCallback, this);
    segmented_map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/segmented_map", 1);
    graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/graph", 1);
    route_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/route", 1);

    num_robot_ = num_robot;
    min_area_ = 20;
    max_area_ = 150;
    region_num = 0;
    loop_ = -1;
    // path_pub_
    path_pub_.resize(num_robot_);
    for (size_t i = 0; i < num_robot_; i++)
    {
        path_pub_[i] = nh_.advertise<nav_msgs::Path>("/tb3_" + std::to_string(i) + "/path", 1);
    }
}

patrol::MapToGraph::~MapToGraph()
{
}

void patrol::MapToGraph::mapCallback(const nav_msgs::OccupancyGrid &map)
{
    loop_ ++;
    if(loop_ % 10 != 0)
    {
        ROS_INFO("wait");
        return;
    } 
    ROS_INFO("Map received");
    // Get map data
    map_data_.reset(new MapData);
    map_data_->height_ = map.info.height;
    map_data_->width_ = map.info.width;
    map_data_->resolution_ = map.info.resolution;
    map_data_->origin_ = {map.info.origin.position.x, map.info.origin.position.y};
    map_data_->map_ = std::vector<std::vector<size_t>>(map_data_->height_, std::vector<size_t>(map_data_->width_, 0));
    ROS_INFO("Map size: %d x %d", map_data_->height_, map_data_->width_);
    for (size_t i = 0; i < map_data_->height_; i++)
    {
        for (size_t j = 0; j < map_data_->width_; j++)
        {
            map_data_->map_[i][j] = map.data[i * map_data_->width_ + j];
        }
    }

    // clear old data
    region_num = 0;
    regions.clear();

    makeSegmentation();
    makeGraph();
    makeRoute();
}

void patrol::MapToGraph::makeSegmentation()
{
    ROS_INFO("Start segmentation");
    // Segment map into different regions
    segmented_map_.reset(new std::vector<std::vector<size_t>>(map_data_->height_, std::vector<size_t>(map_data_->width_, 0)));

    for (size_t i = 0; i < map_data_->height_; i++)
    {
        for (size_t j = 0; j < map_data_->width_; j++)
        {
            if (map_data_->map_[i][j] == 0 && (*segmented_map_)[i][j] == 0)
            {
                region_num++;
                ROS_INFO("Region %d", region_num);
                std::queue<std::array<size_t, 2>> q;
                std::vector<std::array<size_t, 2>> region;
                // CCL algorithm
                // min_area_ and max_area_ are used to filter out small and large regions
                size_t area = 0;
                q.push({i, j});
                while (!q.empty() && area < max_area_)
                {
                    std::array<size_t, 2> current = q.front();
                    q.pop();
                    if (current[0] < 0 || current[0] >= map_data_->height_ || current[1] < 0 || current[1] >= map_data_->width_)
                    {
                        continue;
                    }
                    if (map_data_->map_[current[0]][current[1]] != 0 || (*segmented_map_)[current[0]][current[1]] != 0)
                    {
                        continue;
                    }
                    (*segmented_map_)[current[0]][current[1]] = region_num;
                    area++;
                    region.push_back(current);
                    q.push({current[0] + 1, current[1]});
                    q.push({current[0] - 1, current[1]});
                    q.push({current[0], current[1] + 1});
                    q.push({current[0], current[1] - 1});
                }

                if (area < min_area_)
                {
                    for (size_t k = 0; k < region.size(); k++)
                    {
                        (*segmented_map_)[region[k][0]][region[k][1]] = -1;
                    }
                    region_num--;
                }
                else
                {

                    // arithmetic center of the region
                    double x = 0;
                    double y = 0;
                    for (size_t k = 0; k < region.size(); k++)
                    {
                        x += region[k][1] * map_data_->resolution_ + map_data_->origin_[0];
                        y += region[k][0] * map_data_->resolution_ + map_data_->origin_[1];
                    }
                    x /= region.size();
                    y /= region.size();
                    regions.push_back({region_num, {x, y}});
                }
            }
        }
    }

    ROS_INFO("Region number: %d", region_num);

    // visualize segmented map with different colors
    visualization_msgs::MarkerArray segmented_map_marker;
    segmented_map_marker.markers.resize(region_num);
    for (size_t i = 0; i < region_num; i++)
    {
        segmented_map_marker.markers[i].header.frame_id = "map";
        segmented_map_marker.markers[i].header.stamp = ros::Time::now();
        segmented_map_marker.markers[i].ns = "segmented_map";
        segmented_map_marker.markers[i].id = i;
        segmented_map_marker.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        segmented_map_marker.markers[i].action = visualization_msgs::Marker::ADD;
        segmented_map_marker.markers[i].scale.x = map_data_->resolution_;
        segmented_map_marker.markers[i].scale.y = map_data_->resolution_;
        segmented_map_marker.markers[i].scale.z = map_data_->resolution_;
        segmented_map_marker.markers[i].color.a = 1.0;
        segmented_map_marker.markers[i].color.r = (double)rand() / RAND_MAX;
        segmented_map_marker.markers[i].color.g = (double)rand() / RAND_MAX;
        segmented_map_marker.markers[i].color.b = (double)rand() / RAND_MAX;
    }

    for (size_t i = 0; i < map_data_->height_; i++)
    {
        for (size_t j = 0; j < map_data_->width_; j++)
        {
            if ((*segmented_map_)[i][j] != 0 && (*segmented_map_)[i][j] != -1)
            {
                geometry_msgs::Point p;
                p.x = j * map_data_->resolution_ + map_data_->origin_[0];
                p.y = i * map_data_->resolution_ + map_data_->origin_[1];
                p.z = 0;
                segmented_map_marker.markers[(*segmented_map_)[i][j] - 1].points.push_back(p);
            }
        }
    }

    segmented_map_pub_.publish(segmented_map_marker);
}

void patrol::MapToGraph::makeGraph()
{
    ROS_INFO("Start graph construction");
    // Construct graph
    graph_.reset(new std::vector<std::vector<bool>>(region_num, std::vector<bool>(region_num, false)));

    // find neighbors of each region
    std::vector<std::array<int, 2>> neighbors = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    for (size_t i = 0; i < map_data_->height_ - 1; i++)
    {
        for (size_t j = 0; j < map_data_->width_ - 1; j++)
        {
            auto region_id = (*segmented_map_)[i][j];
            if (region_id == 0 || region_id == -1)
            {
                continue;
            }
            for (size_t k = 0; k < neighbors.size(); k++)
            {
                if (i + neighbors[k][0] < 0 || i + neighbors[k][0] >= map_data_->height_ || j + neighbors[k][1] < 0 || j + neighbors[k][1] >= map_data_->width_)
                {
                    continue;
                }
                auto neighbor_id = (*segmented_map_)[i + neighbors[k][0]][j + neighbors[k][1]];
                if (neighbor_id == 0 || neighbor_id == -1)
                {
                    continue;
                }
                if (region_id != neighbor_id)
                {
                    (*graph_)[region_id - 1][neighbor_id - 1] = true;
                    (*graph_)[neighbor_id - 1][region_id - 1] = true;
                }
            }
        }
    }

    // print region_id and its number of neighbors
    for (size_t i = 0; i < graph_->size(); i++)
    {
        int region_id = std::get<0>(regions[i]);
        int neighbor_num = 0;
        for (size_t j = 0; j < (*graph_)[i].size(); j++)
        {
            if ((*graph_)[i][j])
            {
                neighbor_num++;
            }
        }
        ROS_INFO("Region %d has %d neighbors", region_id, neighbor_num);
    }

    ROS_INFO("Graph construction finished");

    // visualize graph
    // visualize node positions with red spheres
    // visualize edges with blue lines
    visualization_msgs::MarkerArray graph_marker;
    graph_marker.markers.resize(2);
    graph_marker.markers[0].header.frame_id = "map";
    graph_marker.markers[0].header.stamp = ros::Time::now();
    graph_marker.markers[0].ns = "graph";
    graph_marker.markers[0].id = 0;
    graph_marker.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    graph_marker.markers[0].action = visualization_msgs::Marker::ADD;
    graph_marker.markers[0].scale.x = 0.1;
    graph_marker.markers[0].scale.y = 0.1;
    graph_marker.markers[0].scale.z = 0.1;
    graph_marker.markers[0].color.a = 1.0;
    graph_marker.markers[0].color.r = 1.0;
    graph_marker.markers[0].color.g = 0.0;
    graph_marker.markers[0].color.b = 0.0;

    graph_marker.markers[1].header.frame_id = "map";
    graph_marker.markers[1].header.stamp = ros::Time::now();
    graph_marker.markers[1].ns = "graph";
    graph_marker.markers[1].id = 1;
    graph_marker.markers[1].type = visualization_msgs::Marker::LINE_LIST;
    graph_marker.markers[1].action = visualization_msgs::Marker::ADD;
    graph_marker.markers[1].scale.x = 0.01;
    graph_marker.markers[1].color.a = 1.0;
    graph_marker.markers[1].color.r = 0.0;
    graph_marker.markers[1].color.g = 0.0;
    graph_marker.markers[1].color.b = 1.0;

    graph_marker.markers[2].color.b = 1.0;

    for (size_t i = 0; i < graph_->size(); i++)
    {
        geometry_msgs::Point p;
        std::array<double, 2> center = std::get<1>(regions[i]);
        size_t region_id = std::get<0>(regions[i]);
        p.x = center[0];
        p.y = center[1];
        p.z = 1;
        graph_marker.markers[0].points.push_back(p);

        // text push back

        for (size_t j = 0; j < (*graph_)[i].size(); j++)
        {
            if ((*graph_)[i][j])
            {
                geometry_msgs::Point p2;
                std::array<double, 2> center2 = std::get<1>(regions[j]);
                p2.x = center2[0];
                p2.y = center2[1];
                p2.z = 1;
                graph_marker.markers[1].points.push_back(p);
                graph_marker.markers[1].points.push_back(p2);
            }
        }
    }

    graph_pub_.publish(graph_marker);

    ROS_INFO("Graph visualization finished");
}

void patrol::MapToGraph::makeRoute()
{
    size_t route_num = regions.size() / num_robot_;
    ROS_INFO("Route number: %d", route_num);

    std::array<int, 2> head{0, 0};

    std::queue<int> q;
    std::vector<int> v;

    q.push({0});
    v.push_back({0});
    auto traversed = *graph_;
    while (!q.empty())
    {
        int head = q.front();
        q.pop();
        for (int i = 0; i < graph_->size(); i++)
        {
            if ((*graph_)[head][i] && traversed[head][i])
            {
                traversed[head][i] = false;
                bool visited = false;
                for (int x = 0; x < v.size(); x++)
                {
                    if (v[x] == i)
                    {
                        visited = true;
                    }
                }
                if (!visited)
                {
                    q.push(i);
                    v.push_back(i);
                    ROS_INFO_STREAM(head << " - " << i);
                }
            }
        }
    }

    ROS_INFO("vector size: %d", v.size());
    std::string s;
    for (int i = 0; i < v.size(); i++)
    {
        s += std::to_string(v[i]);
        s += " ";
    }
    ROS_INFO_STREAM(s);

    // construct routes
    std::vector<std::vector<geometry_msgs::PoseStamped>> routes;
    routes.resize(num_robot_);

    int j = 0;
    for (int i = 0; i < num_robot_; i++)
    {
        if (v.size() - j > route_num)
        {
            routes[i].resize(route_num);
        }
        else
        {
            routes[i].resize(v.size() - j);
        }
        int size = routes[i].size();
        ROS_INFO_STREAM(routes[i].size());
        for (int x = 0; x < size; x++)
        {
            ROS_INFO_STREAM("index: " << j);
            geometry_msgs::PoseStamped p;
            std::array<double, 2> pose = std::get<1>(regions[j]);
            p.pose.position.x = pose[0];
            p.pose.position.y = pose[1];
            p.pose.position.z = 0;
            routes[i][x] = p;
            ROS_INFO_STREAM("x: " << pose[0] << " y: " << pose[1]);
            j++;
        }
        ROS_INFO_STREAM(j);
    }

    ROS_INFO("Route construction finished");

    // Optimize routes

    for (size_t i = 0; i < num_robot_; i++)
    {
    }

    // visualize routes
    visualization_msgs::MarkerArray route_marker;
    route_marker.markers.resize(num_robot_);
    for (size_t i = 0; i < num_robot_; i++)
    {
        route_marker.markers[i].header.frame_id = "map";
        route_marker.markers[i].header.stamp = ros::Time::now();
        route_marker.markers[i].ns = "route";
        route_marker.markers[i].id = i;
        route_marker.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        route_marker.markers[i].action = visualization_msgs::Marker::ADD;
        route_marker.markers[i].scale.x = 0.01;
        route_marker.markers[i].color.a = 1.0;
        route_marker.markers[i].color.r = (double)rand() / RAND_MAX;
        route_marker.markers[i].color.g = (double)rand() / RAND_MAX;
        route_marker.markers[i].color.b = (double)rand() / RAND_MAX;

        for (size_t j = 0; j < routes[i].size(); j++)
        {
            geometry_msgs::Point p;
            p.x = routes[i][j].pose.position.x;
            p.y = routes[i][j].pose.position.y;
            p.z = 1;
            route_marker.markers[i].points.push_back(p);
        }
    }

    route_pub_.publish(route_marker);

    ROS_INFO("Route visualization finished");

    // publish routes
    std::vector<nav_msgs::Path> paths;
    paths.resize(num_robot_);
    for (size_t i = 0; i < num_robot_; i++)
    {
        paths[i].header.frame_id = "map";
        paths[i].header.stamp = ros::Time::now();
        for (size_t j = 0; j < routes[i].size(); j++)
        {
            paths[i].poses.push_back(routes[i][j]);
        }
    }

    for (size_t i = 0; i < num_robot_; i++)
    {
        path_pub_[i].publish(paths[i]);
    }

    ROS_INFO("Route publishing finished");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_graph");

    ros::NodeHandle nh;
    size_t num_robot = 3;
    patrol::MapToGraph map_to_graph(nh, num_robot);

    ros::spin();
    return 0;
}