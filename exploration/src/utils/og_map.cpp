#include <exploration/utils/og_map.h>

namespace mre
{
    OGMap::OGMap(nav_msgs::OccupancyGrid map) : height(map.info.height), width(map.info.width), resolution(map.info.resolution), origin(std::make_pair(map.info.origin.position.x, map.info.origin.position.y))
    {
        map_data.resize(height, std::vector<int>(width, 0));
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                map_data[i][j] = map.data[i * width + j];
            }
        }
        calculateFrontiers();
    }

    void OGMap::calculateFrontiers()
    {
        std::vector<std::array<int, 2>> neighbors{
            {-1, -1},
            {1, 1},
            {0, 1},
            {1, 0},
            {0, -1},
            {-1, 0},
            {1, -1},
            {-1, 1}};
        frontier.resize(height, std::vector<bool>(width, false));
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                if (isFree(std::make_pair(i, j)))
                {
                    auto it = neighbors.cbegin();
                    while (it != neighbors.cend())
                    {
                        auto neighbor_p = std::make_pair<int, int>(i + it->at(0), j + it->at(1));
                        if (inBoundary(neighbor_p) && isUnknown(neighbor_p))
                        {
                            frontier[i][j] = true;
                            break;
                        }
                        it++;
                    }
                }
            }
        }
    }

    bool OGMap::isFree(std::pair<int, int> index)
    {
        return map_data[index.first][index.second] == 0;
    }

    bool OGMap::isFrontier(std::pair<int, int> index)
    {
        return frontier[index.first][index.second];
    }

    bool OGMap::isUnknown(std::pair<int, int> index)
    {
        return map_data[index.first][index.second] == -1;
    }

    bool OGMap::inBoundary(std::pair<int, int> index)
    {
        return index.first >= 0 && index.first < height && index.second >= 0 && index.second < width;
    }

} // namespace mre