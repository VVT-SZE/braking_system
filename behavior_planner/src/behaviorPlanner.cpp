#include <behavior_planner/behaviorPlanner.hpp>

brakingSystem::BehaviorPlanner::BehaviorPlanner() : Node("behavior_planner")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_scenario", "scenario");
    this->declare_parameter<std::string>("output_topic_scenario", "plan/strategy");
    this->declare_parameter<std::string>("output_topic_target_space", "plan/target_space");
    this->declare_parameter<bool>("debug_enabled", false);
    this->declare_parameter<double>("critical_distance", 50.0);

    std::string inputTopicEgo;
    std::string inputTopicScenario;
    std::string outputTopicScenario;
    std::string outputTopicTargetSpace;

    this->get_parameter<std::string>("input_topic_ego", inputTopicEgo);
    this->get_parameter<std::string>("input_topic_scenario", inputTopicScenario);
    this->get_parameter<std::string>("output_topic_scenario", outputTopicScenario);
    this->get_parameter<std::string>("output_topic_target_space", outputTopicTargetSpace);
    this->get_parameter<double>("critical_distance", m_critical_distance_);


    m_subScenario_ = this->create_subscription<crp_msgs::msg::Scenario>(
        inputTopicScenario,
        1,
        std::bind(&BehaviorPlanner::scenarioCallback, this, std::placeholders::_1));

    m_subEgo_ = this->create_subscription<crp_msgs::msg::Ego>(
        inputTopicEgo,
        1,
        std::bind(&BehaviorPlanner::egoCallback, this, std::placeholders::_1));

    m_pubScenario_ = this->create_publisher<tier4_planning_msgs::msg::Scenario>(
        outputTopicScenario,
        1);

    m_pubTargetSpace_ = this->create_publisher<crp_msgs::msg::TargetSpace>(
        outputTopicTargetSpace,
        1);

    RCLCPP_INFO(this->get_logger(), "behavior_planner node has been started (critical_distance=%.2f)", m_critical_distance_);
}

void brakingSystem::BehaviorPlanner::scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg)
{
    const float LANE_WIDTH = 3.5f;
    this->get_parameter<double>("critical_distance", m_critical_distance_);

    int closest_id = -1;
    float closest = std::numeric_limits<float>::max();

    std::vector<std::pair<double, double>> critical_objects_x_y;

    int counter = 0;

    for (const auto &obj : msg->local_moving_objects.objects)
    {
        double obj_x = obj.kinematics.initial_pose_with_covariance.pose.position.x;
        double obj_y = obj.kinematics.initial_pose_with_covariance.pose.position.y;

        if (obj_x > 0 && obj_x < m_critical_distance_ && std::abs(obj_y) < LANE_WIDTH / 2.0f)
        {
            critical_objects_x_y.push_back(std::make_pair(obj_x, obj_y));
            if (closest > obj_x)
            {
                closest = obj_x;
                closest_id = counter;
            }
            counter++;
        }
    }
    if (!critical_objects_x_y.empty())
    {
        std::pair<double, double> critical_obj = critical_objects_x_y[closest_id];
        RCLCPP_INFO(this->get_logger(), "Closest object X: %f, Y: %f", critical_obj.first, critical_obj.second);
    }
}

void brakingSystem::BehaviorPlanner::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    m_ego_pose_ = msg->pose.pose.position;
    m_ego_heading_ = msg->orientation;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<brakingSystem::BehaviorPlanner>());
    rclcpp::shutdown();
    return 0;
}