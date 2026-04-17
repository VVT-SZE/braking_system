#include <behavior_planner/behaviorPlanner.hpp>

brakingSystem::BehaviorPlanner::BehaviorPlanner() : Node("behavior_planner")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_scenario", "scenario");
    this->declare_parameter<std::string>("output_topic_scenario", "plan/strategy");
    this->declare_parameter<std::string>("output_topic_target_space", "plan/target_space");
    this->declare_parameter<bool>("debug_enabled", false);
    this->declare_parameter<double>("critical_distance", 50.0);
    this->declare_parameter<double>("publish_rate", 50.0);
    this->declare_parameter<double>("warning_threshold", 1.5);
    this->declare_parameter<double>("emergency_threshold", 2.5);

    std::string inputTopicEgo;
    std::string inputTopicScenario;
    std::string outputTopicScenario;
    std::string outputTopicTargetSpace;
    double warning_threshold;
    double emergency_threshold;
    double frequency;

    this->get_parameter<std::string>("input_topic_ego", inputTopicEgo);
    this->get_parameter<std::string>("input_topic_scenario", inputTopicScenario);
    this->get_parameter<std::string>("output_topic_scenario", outputTopicScenario);
    this->get_parameter<std::string>("output_topic_target_space", outputTopicTargetSpace);
    this->get_parameter<double>("critical_distance", m_critical_distance_);
    this->get_parameter<double>("publish_rate", frequency);
    this->get_parameter<double>("warning_threshold", warning_threshold);
    this->get_parameter<double>("emergency_threshold", emergency_threshold);

    TrajectoryCalculator = new TrajectoryCalculation::TrajectoryCalculation(m_critical_distance_);

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

    if (frequency <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid publish_rate (%.3f). Falling back to 10.0 Hz.", frequency);
        frequency = 10.0;
    }

    auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / frequency));
    if (timer_period.count() < 1)
    {
        timer_period = std::chrono::milliseconds(1);
    }

    m_timer_ = this->create_wall_timer(timer_period, std::bind(&brakingSystem::BehaviorPlanner::run, this));

    RCLCPP_INFO(this->get_logger(), "behavior_planner node has been started (critical_distance=%.2f, publish_rate=%.2f)", m_critical_distance_, frequency);
}

void brakingSystem::BehaviorPlanner::scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg)
{
    // TODO: calculate distance based on left and right lane distance from each other.
    const float LANE_WIDTH = 4.0f;
    double width = std::abs(msg->paths[0].left_bound.front().y - msg->paths[0].right_bound.front().y);
    if (width < LANE_WIDTH)
    {
        RCLCPP_INFO(this->get_logger(), "Lane width (%.2f m) is less than expected (%.2f m).", width, LANE_WIDTH);
        LANE_WIDTH = width;
    }
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
    m_critical_objects_ = critical_objects;
}

void brakingSystem::BehaviorPlanner::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    m_ego_pose_ = msg->pose.pose;
    m_ego_twist_ = msg->twist.twist;
    m_ego_accel_ = msg->accel.accel;
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

        double max_acceleration = TrajectoryCalculator.getMaximumTrajectoryAcceleration(
            closest_obj->kinematics.initial_pose_with_covariance.pose.position.x,
            closest_obj->kinematics.initial_twist_with_covariance.twist.linear.x,
            closest_obj->kinematics.initial_accel_with_covariance.accel.linear.x,
            m_ego_twist_.linear.x,
            m_ego_accel_.linear.x);
        double closest_x = closest_obj->kinematics.initial_pose_with_covariance.pose.position.x;
        double closest_y = closest_obj->kinematics.initial_pose_with_covariance.pose.position.y;

        if (max_acceleration > 9.8) // LONG_EMERGENCY_IMPACT
        {
            RCLCPP_ERROR(this->get_logger(), "Critical object detected within %f meters. Max acceleration: %f", m_critical_distance_, max_acceleration);
            scenario_msg.current_scenario = "LONG_EMERGENCY_IMPACT";
        }
        else if (max_acceleration > emergency_threshold) // LONG_EMERGENCY_AVOID
        {
            RCLCPP_ERROR(this->get_logger(), "Critical object detected within %f meters. Max acceleration: %f", m_critical_distance_, max_acceleration);
            scenario_msg.current_scenario = "LONG_EMERGENCY_AVOID";
        }
        else if (max_acceleration > warning_threshold) // WARNING
        {
            RCLCPP_INFO(this->get_logger(), "Critical object detected within %f meters. Max acceleration: %f", m_critical_distance_, max_acceleration);
            scenario_msg.current_scenario = "WARNING";
        }
        else // NO_ACTION
        {
            scenario_msg.current_scenario = "NO_ACTION";
        }

        if (scenario_msg.current_scenario != "NO_ACTION")
        {
            target_space_msg.header.stamp = this->now();
            target_space_msg.header.frame_id = "base_link";
            target_space_msg.target_pose = closest_obj->kinematics.initial_pose_with_covariance;
            target_space_msg.relevant_objects.push_back(*closest_obj);
        }
        else
        {
            target_space_msg.header.stamp = this->now();
            target_space_msg.header.frame_id = "base_link";
        }
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