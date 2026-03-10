#include <plan_long_emergency/motionHandler.hpp>

brakingSystem::MotionHandler::MotionHandler() : Node("plan_long_emergency")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_scenario", "scenario");
    this->declare_parameter<std::string>("input_topic_target_space", "plan/target_space");
    this->declare_parameter<std::string>("output_topic_trajectory", "plan/longEmergency/trajectory");
    this->declare_parameter<bool>("debug_enabled", false);

    std::string inputTopicEgo;
    std::string inputTopicScenario;
    std::string inputTopicTargetSpace;
    std::string outputTopicTrajectory;

    this->get_parameter("input_topic_ego", inputTopicEgo);
    this->get_parameter("input_topic_scenario", inputTopicScenario);
    this->get_parameter("input_topic_target_space", inputTopicTargetSpace);
    this->get_parameter("output_topic_trajectory", outputTopicTrajectory);

    m_subScenario_ = this->create_subscription<tier4_planning_msgs::msg::Scenario>(
        inputTopicScenario,
        1,
        std::bind(&MotionHandler::scenarioCallback, this, std::placeholders::_1));

    m_subEgo_ = this->create_subscription<crp_msgs::msg::Ego>(
        inputTopicEgo,
        1,
        std::bind(&MotionHandler::egoCallback, this, std::placeholders::_1));

    m_subTargetSpace_ = this->create_subscription<crp_msgs::msg::TargetSpace>(
        inputTopicTargetSpace,
        1,
        std::bind(&MotionHandler::targetSpaceCallback, this, std::placeholders::_1));

    m_pubTrajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
        outputTopicTrajectory,
        1);

    RCLCPP_INFO(this->get_logger(), "Plan_long_emergency node has been started");
}


void brakingSystem::MotionHandler::scenarioCallback(const tier4_planning_msgs::msg::Scenario::SharedPtr msg)
{
    // Implementation for scenario callback
}


void brakingSystem::MotionHandler::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    // Implementation for ego callback
}


void brakingSystem::MotionHandler::targetSpaceCallback(const crp_msgs::msg::TargetSpace::SharedPtr msg)
{
    // Implementation for targetSpace callback
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brakingSystem::MotionHandler>());
  rclcpp::shutdown();
  return 0;
}