#include <plan_ctrl_long_emergency/ctrlLongEmergency.hpp>

crp::apl::CtrlLongEmergency ::CtrlLongEmergency () : Node("ctrl_long_emergency")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_trajectory", "trajectory");
    this->declare_parameter<std::string>("output_topic_control", "control/command/control_cmd");
    this->declare_parameter<bool>("debug_enabled", false);

    std::string inputTopicEgo;
    std::string inputTopicTajectory;
    std::string outputTopicControl;

    m_subEgo_ = this->create_subscription<crp_msgs::msg::ego>(
        inputTopicEgo,
        1,
        std::bind(&CtrlLongEmergency::egoCallback, this, std::placeholders::_1));

    m_subTrajectory_ = this->create_publisher<crp_msgs::msg::Trajectory>(
        inputTopicTrajectory
        1,
        std::bind(&CtrlLongEmergency::trajectoryCallback, this, std::placeholders::_1));

    m_pubControl = this->create_publisher<autoware_control_msgs::msg::Control>(
        outputTopicControl,
        1);

    RCLCPP_INFO(this->get_logger(), "ctrl_long_emergency node has been started");
}

void brakingSystem::CtrlLongEmergency::egoCallback(const crp_msgs::msg::ego::SharedPtr msg)
{
    // Implementation for ego callback
}

void brakingSystem::CtrlLongEmergency::trajectoryCallback(const crp_msgs::msg::trajectory::SharedPtr msg)
{
    // Implementation for trajectory callback
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brakingSystem::CtrlLongEmergency>());
  rclcpp::shutdown();
  return 0;
}