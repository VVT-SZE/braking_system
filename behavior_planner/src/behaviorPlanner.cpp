#include <behavior_planner/behaviorPlanner.hpp>

brakingSystem::BehaviorPlanner::BehaviorPlanner() : rclcpp::Node("behavior_planner")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_scenario", "scenario");
    this->declare_parameter<std::string>("output_topic_scenario", "plan/strategy");
    this->declare_parameter<std::string>("output_topic_target_space", "plan/target_space");
    this->declare_parameter<bool>("debug_enabled", false);

    std::string inputTopicEgo;
    std::string inputTopicScenario;
    std::string outputTopicScenario;
    std::string outputTopicTargetSpace;

    this->get_parameter<std::string>("input_topic_ego", inputTopicEgo);
    this->get_parameter<std::string>("input_topic_scenario", inputTopicScenario);
    this->get_parameter<std::string>("output_topic_scenario", outputTopicScenario);
    this->get_parameter<std::string>("output_topic_target_space", outputTopicTargetSpace);

    m_subScenario_ = this->create_subscription<crp_msgs::msg::scenario>(
        inputTopicScenario,
        1,
        std::bind(&BehaviorPlanner::scenarioCallback, this, std::placeholders::_1));

    m_subEgo_ = this->create_subscription<crp_msgs::msg::ego>(
        inputTopicEgo,
        1,
        std::bind(&BehaviorPlanner::egoCallback, this, std::placeholders::_1));

    m_pubScenario_ = this->create_publisher<tier4_planning_msgs::msg::Scenario>(
        outputTopicScenario,
        1);

    m_pubTargetSpace_ = this->create_publisher<crp_msgs::msg::TargetSpace>(
        outputTopicTargetSpace,
        1);

    RCLCPP_INFO(this->get_logger(), "behavior_planner node has been started");
}

void brakingSystem::BehaviorPlanner::scenarioCallback(const crp_msgs::msg::scenario::SharedPtr msg)
{
    // Implementation for scenario callback
}

void brakingSystem::BehaviorPlanner::egoCallback(const crp_msgs::msg::ego::SharedPtr msg)
{
    // Implementation for ego callback
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<brakingSystem::BehaviorPlanner>());
    rclcpp::shutdown();
    return 0;
}