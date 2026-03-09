#include <ctrl_long_emergency/ctrlLongEmergency.hpp>

brakingSystem::CtrlLongEmergency ::CtrlLongEmergency () : Node("ctrl_long_emergency")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_trajectory", "trajectory");
    this->declare_parameter<std::string>("output_topic_control", "control/command/control_cmd");
    this->declare_parameter<bool>("debug_enabled", false);

    std::string inputTopicEgo;
    std::string inputTopicTrajectory;
    std::string outputTopicControl;

    this->get_parameter("input_topic_ego", inputTopicEgo);
    this->get_parameter("input_topic_trajectory", inputTopicTrajectory);
    this->get_parameter("output_topic_control", outputTopicControl);

    m_subEgo_ = this->create_subscription<crp_msgs::msg::Ego>(
        inputTopicEgo,
        1,
        std::bind(&CtrlLongEmergency::egoCallback, this, std::placeholders::_1));

    m_subTrajectory_ = this->create_subscription<tier4_planning_msgs::msg::Trajectory>(
        inputTopicTrajectory,
        1,
        std::bind(&CtrlLongEmergency::trajectoryCallback, this, std::placeholders::_1));

    m_pubControl_ = this->create_publisher<autoware_control_msgs::msg::Control>(
        outputTopicControl,
        1);

    RCLCPP_INFO(this->get_logger(), "ctrl_long_emergency node has been started");
}

void brakingSystem::CtrlLongEmergency::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    // Implementation for ego callback
}

void brakingSystem::CtrlLongEmergency::trajectoryCallback(const tier4_planning_msgs::msg::Trajectory::SharedPtr msg)
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