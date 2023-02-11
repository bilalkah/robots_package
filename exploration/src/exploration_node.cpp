#include <exploration/exploration_node.h>

namespace mre
{
    ExplorationNode::ExplorationNode(ros::NodeHandle &nh) : nh_(nh)
    {
        std::string control_strategy("topic");
        double tolerance = 1.0;
        int min_cluster = 15;
        std::vector<std::string> robot_names{};

        nh_.getParam("/exploration_node/control_logic", control_strategy);
        nh_.getParam("/exploration_node/tolerance", tolerance);
        nh_.getParam("/exploration_node/min_cluster", min_cluster);
        XmlRpc::XmlRpcValue name_list;
        nh_.getParam("/exploration_node/robot_name", name_list);
        ROS_ASSERT(name_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < name_list.size(); i++)
        {
            robot_names.push_back(name_list[i]);
        }
        v_robot_ptr_.resize(robot_names.size());

        exploration_points_ptr_ = std::make_shared<TSVector<geometry_msgs::PoseStamped>>();
        exp_ptr_ = std::make_unique<Exploration>(nh_, exploration_points_ptr_, min_cluster);
        for (int i = 0; i < robot_names.size(); i++)
        {
            if (control_strategy == "topic")
            {
                RobotControlLogic *rcl_ptr = new TopicLogic(nh_, robot_names[i], tolerance);
                v_robot_ptr_[i] = std::make_unique<Robot>(rcl_ptr, exploration_points_ptr_);
            }
            else if (control_strategy == "service")
            {
                RobotControlLogic *rcl_ptr = new ServiceLogic(nh_, robot_names[i], tolerance);
                v_robot_ptr_[i] = std::make_unique<Robot>(rcl_ptr, exploration_points_ptr_);
            }
        }
    }

    ExplorationNode::~ExplorationNode()
    {
    }
} // namespace mre

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_node");
    ros::NodeHandle nh;

    mre::ExplorationNode node(nh);
    ros::spin();
    return 0;
}