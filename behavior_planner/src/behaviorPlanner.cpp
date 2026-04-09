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
    double frequency;

    this->get_parameter<std::string>("input_topic_ego", inputTopicEgo);
    this->get_parameter<std::string>("input_topic_scenario", inputTopicScenario);
    this->get_parameter<std::string>("output_topic_scenario", outputTopicScenario);
    this->get_parameter<std::string>("output_topic_target_space", outputTopicTargetSpace);
    this->get_parameter<double>("critical_distance", m_critical_distance_);
    this->get_parameter<double>("publish_rate", frequency);

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

    m_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / frequency), std::bind(&brakingSystem::BehaviorPlanner::run, this));

    RCLCPP_INFO(this->get_logger(), "behavior_planner node has been started (critical_distance=%.2f)", m_critical_distance_);
}

void brakingSystem::BehaviorPlanner::scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg)
{
    // TODO: calculate distance based on left and right lane distance from each other.
    const float LANE_WIDTH = 3.5f;
    this->get_parameter<double>("critical_distance", m_critical_distance_);

    std::vector<autoware_perception_msgs::msg::PredictedObject> critical_objects;
    // Find critical objects within the specified distance and lane width and store the whole object instead of just the position
    for (const auto &obj : msg->local_moving_objects.objects)
    {
        double obj_x = obj.kinematics.initial_pose_with_covariance.pose.position.x;
        double obj_y = obj.kinematics.initial_pose_with_covariance.pose.position.y;

        if (obj_x > 0 && obj_x < m_critical_distance_ && std::abs(obj_y) < LANE_WIDTH / 2.0f)
        {
            critical_objects.push_back(obj);
        }
    }
    m_critical_objects_.clear();
    m_critical_objects_ = critical_objects;
}

void brakingSystem::BehaviorPlanner::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    m_ego_pose_ = msg->pose.pose.position;
    m_ego_heading_ = msg->orientation;
}

void brakingSystem::BehaviorPlanner::run()
{
    auto target_space_msg = crp_msgs::msg::TargetSpace();
    auto scenario_msg = tier4_planning_msgs::msg::Scenario();
    // TODO: implement the logic to determine the scenario and target space based on the critical objects and ego state. For now, we will just publish the closest critical object if it exists.
    if (!m_critical_objects_.empty())
    {
        auto closest_obj = std::min_element(
            m_critical_objects_.begin(),
            m_critical_objects_.end(),
            [](const autoware_perception_msgs::msg::PredictedObject &a, const autoware_perception_msgs::msg::PredictedObject &b)
            {
                return a.kinematics.initial_pose_with_covariance.pose.position.x <
                       b.kinematics.initial_pose_with_covariance.pose.position.x;
            });

        double closest_x = closest_obj->kinematics.initial_pose_with_covariance.pose.position.x;
        double closest_y = closest_obj->kinematics.initial_pose_with_covariance.pose.position.y;

        RCLCPP_INFO(this->get_logger(), "Closest object X: %f, Y: %f", closest_x, closest_y);

        target_space_msg.header.stamp = this->now();
        target_space_msg.header.frame_id = "base_link";
        target_space_msg.target_pose = closest_obj->kinematics.initial_pose_with_covariance;
        target_space_msg.relevant_objects.push_back(*closest_obj);
        scenario_msg.current_scenario = "LONG_EMERGENCY_IMPACT";
    }
    else
    {
        target_space_msg.header.stamp = this->now();
        target_space_msg.header.frame_id = "base_link";
        scenario_msg.current_scenario = "NO_ACTION";
    }

    m_pubScenario_->publish(scenario_msg);
    m_pubTargetSpace_->publish(target_space_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<brakingSystem::BehaviorPlanner>());
    rclcpp::shutdown();
    return 0;
}