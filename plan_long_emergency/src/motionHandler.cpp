#include <plan_long_emergency/motionHandler.hpp>

brakingSystem::MotionHandler::MotionHandler() : Node("plan_long_emergency")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_scenario", "plan/strategy");
    this->declare_parameter<std::string>("input_topic_target_space", "plan/target_space");
    this->declare_parameter<std::string>("output_topic_trajectory", "plan/longEmergency/trajectory");
    this->declare_parameter<double>("safety_distance", 5.0);

    std::string inputTopicEgo, inputTopicScenario, inputTopicTargetSpace, outputTopicTrajectory;
    double safety_distance;
    this->get_parameter("input_topic_ego", inputTopicEgo);
    this->get_parameter("input_topic_scenario", inputTopicScenario);
    this->get_parameter("input_topic_target_space", inputTopicTargetSpace);
    this->get_parameter("output_topic_trajectory", outputTopicTrajectory);
    this->get_parameter("safety_distance", safety_distance);

    m_subScenario_ = this->create_subscription<tier4_planning_msgs::msg::Scenario>(
        inputTopicScenario, 1, std::bind(&MotionHandler::scenarioCallback, this, std::placeholders::_1));

    m_subEgo_ = this->create_subscription<crp_msgs::msg::Ego>(
        inputTopicEgo, 1, std::bind(&MotionHandler::egoCallback, this, std::placeholders::_1));

    m_subTargetSpace_ = this->create_subscription<crp_msgs::msg::TargetSpace>(
        inputTopicTargetSpace, 1, std::bind(&MotionHandler::targetSpaceCallback, this, std::placeholders::_1));

    m_pubTrajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
        outputTopicTrajectory, 1);

    m_current_scenario_ = "NO_ACTION";

    trajectoryCalculator = new brakingSystem::TrajectoryCalculation(safety_distance);
    

    RCLCPP_INFO(this->get_logger(), "Plan_long_emergency node has been started.");
}

void brakingSystem::MotionHandler::scenarioCallback(const tier4_planning_msgs::msg::Scenario::SharedPtr msg)
{
    // Log only if the scenario actually changes
    if (m_current_scenario_ != msg->current_scenario) {
        m_current_scenario_ = msg->current_scenario;
        RCLCPP_INFO(this->get_logger(), "Strategy changed to: %s", m_current_scenario_.c_str());
    }
    if (m_current_scenario_ == "NO_ACTION" || m_current_scenario_ == "WARNING") {
        autoware_planning_msgs::msg::Trajectory trajectory_msg;
        m_pubTrajectory_->publish(trajectory_msg);
        RCLCPP_INFO(this->get_logger(), "No relevant objects");
    }
}

void brakingSystem::MotionHandler::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{   
    (void)msg;
}

void brakingSystem::MotionHandler::targetSpaceCallback(const crp_msgs::msg::TargetSpace::SharedPtr msg)
{
    // deciding if there is a critical object or is an emergeny
    bool is_emergency = (m_current_scenario_ == "LONG_EMERGENCY_AVOID" || 
        m_current_scenario_ == "LONG_EMERGENCY_IMPACT");
        
    if (is_emergency) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Obstacle detected! Object count: %zu", msg->relevant_objects.size());
    }
        
    // getting x and y from behaviour planner
    const auto &obj = msg->relevant_objects[0];
    double obj_x = msg->target_pose.pose.position.x;
    double obj_y = msg->target_pose.pose.position.y;

    RCLCPP_INFO(this->get_logger(), "Object at X: %f, Y: %f", obj_x, obj_y);

    // making new trajectory with vector
    const std::vector<std::vector<double>> trajectory = trajectoryCalculator->calcTrajectory(obj_x, 0.0, 0.0, 0.0, 0.0);

    // convert to ros message
    autoware_planning_msgs::msg::Trajectory trajectory_msg;
    trajectory_msg.header = msg->header;

    for (const std::vector<double> &p : trajectory)
    {
        autoware_planning_msgs::msg::TrajectoryPoint tp;

        tp.pose.position.x = p[0];
        tp.pose.position.y = p[1];
        tp.longitudinal_velocity_mps = p[2];

        trajectory_msg.points.push_back(tp);
    }

    // publish trajectory
    m_pubTrajectory_->publish(trajectory_msg);

    RCLCPP_INFO(this->get_logger(), "Published emergency braking trajectory");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brakingSystem::MotionHandler>());
  rclcpp::shutdown();
  return 0;
}