#include <exploration/exploration_node.h>

namespace mre
{
    ExplorationNode::ExplorationNode(ros::NodeHandle &nh) : nh_(nh)
    {
        std::string control_strategy("service");
        double tolerance = 1.0;
        std::vector<std::string> robot_names{"tb3_0", "tb3_1", "tb3_2", "tb3_3"};
        v_robot_ptr_.resize(robot_names.size());

        exploration_points_ptr_ = new TSVector<geometry_msgs::PoseStamped>();
        // exp_ptr_ = new Exploration(nh_, exploration_points_ptr_);
        exp_ptr_ = std::make_unique<Exploration>(nh_, exploration_points_ptr_);
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
        delete exploration_points_ptr_;
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