#include <plan_long_emergency/motionHandler.hpp>

brakingSystem::MotionHandler::MotionHandler() : Node("plan_long_emergency")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_scenario", "plan/strategy");
    this->declare_parameter<std::string>("input_topic_target_space", "plan/target_space");
    this->declare_parameter<std::string>("output_topic_trajectory", "plan/longEmergency/trajectory");

    std::string inputTopicEgo, inputTopicScenario, inputTopicTargetSpace, outputTopicTrajectory;
    this->get_parameter("input_topic_ego", inputTopicEgo);
    this->get_parameter("input_topic_scenario", inputTopicScenario);
    this->get_parameter("input_topic_target_space", inputTopicTargetSpace);
    this->get_parameter("output_topic_trajectory", outputTopicTrajectory);

    m_subScenario_ = this->create_subscription<tier4_planning_msgs::msg::Scenario>(
        inputTopicScenario, 1, std::bind(&MotionHandler::scenarioCallback, this, std::placeholders::_1));

    m_subEgo_ = this->create_subscription<crp_msgs::msg::Ego>(
        inputTopicEgo, 1, std::bind(&MotionHandler::egoCallback, this, std::placeholders::_1));

    m_subTargetSpace_ = this->create_subscription<crp_msgs::msg::TargetSpace>(
        inputTopicTargetSpace, 1, std::bind(&MotionHandler::targetSpaceCallback, this, std::placeholders::_1));

    m_pubTrajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
        outputTopicTrajectory, 1);

    m_current_scenario_ = "NO_ACTION";

    RCLCPP_INFO(this->get_logger(), "Plan_long_emergency node has been started.");
}

void brakingSystem::MotionHandler::scenarioCallback(const tier4_planning_msgs::msg::Scenario::SharedPtr msg)
{
    // Log only if the scenario actually changes
    if (m_current_scenario_ != msg->current_scenario) {
        m_current_scenario_ = msg->current_scenario;
        RCLCPP_INFO(this->get_logger(), "Strategy changed to: %s", m_current_scenario_.c_str());
    }
}

void brakingSystem::MotionHandler::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{   
    (void)msg;
}

void brakingSystem::MotionHandler::targetSpaceCallback(const crp_msgs::msg::TargetSpace::SharedPtr msg)
{
    autoware_planning_msgs::msg::Trajectory trajectory;
    trajectory.header = msg->header;

    bool is_emergency = (m_current_scenario_ == "LONG_EMERGENCY_AVOID" || 
                         m_current_scenario_ == "LONG_EMERGENCY_IMPACT");

    if (is_emergency && !msg->relevant_objects.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Obstacle detected! Object count: %zu", msg->relevant_objects.size());
    }

    double min_distance = 1.0; 
    double last_x =  0.0;
    double last_y = 0.0;
    bool first_point = true;

    for (const auto & path_point : msg->path.path.points)
    {
        double x = path_point.point.pose.position.x;
        double y = path_point.point.pose.position.y;
        double dist = std::hypot(x - last_x, y - last_y);

        if (first_point || dist >= min_distance)
        {
             autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
             trajectory_point.pose = path_point.point.pose;

             if (is_emergency) {
                trajectory_point.longitudinal_velocity_mps = 0.0;
             } else {
                trajectory_point.longitudinal_velocity_mps = path_point.point.longitudinal_velocity_mps;
             }
                
             trajectory.points.push_back(trajectory_point);

             last_x = x;
             last_y = y;
             first_point = false;
        }   
    }

    m_pubTrajectory_->publish(trajectory);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brakingSystem::MotionHandler>());
  rclcpp::shutdown();
  return 0;
}