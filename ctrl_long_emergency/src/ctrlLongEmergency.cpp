#include <ctrl_long_emergency/ctrlLongEmergency.hpp>

brakingSystem::CtrlLongEmergency ::CtrlLongEmergency () : Node("ctrl_long_emergency")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_trajectory", "plan/longEmergency/trajectory");
    this->declare_parameter<std::string>("output_topic_control", "control/command/control_cmd");
    this->declare_parameter<bool>("debug_enabled", false);
    this->declare_parameter<int>("publish_rate", 20);
    this->declare_parameter<double>("m_delay_time", 0.03);

    std::string inputTopicEgo;
    std::string inputTopicTrajectory;
    std::string outputTopicControl;
    int publish_rate;
    double m_delay_time;

    this->get_parameter("input_topic_ego", inputTopicEgo);
    this->get_parameter("input_topic_trajectory", inputTopicTrajectory);
    this->get_parameter("output_topic_control", outputTopicControl);
    this->get_parameter("publish_rate", publish_rate);
    this->get_parameter("m_delay_time", m_delay_time);

    m_subEgo_ = this->create_subscription<crp_msgs::msg::Ego>(
        inputTopicEgo,
        1,
        std::bind(&CtrlLongEmergency::egoCallback, this, std::placeholders::_1));

    m_subTrajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        inputTopicTrajectory,
        1,
        std::bind(&CtrlLongEmergency::trajectoryCallback, this, std::placeholders::_1));

    m_pubControl_ = this->create_publisher<autoware_control_msgs::msg::Control>(
        outputTopicControl,
        1);

    m_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&brakingSystem::CtrlLongEmergency::run, this));

    RCLCPP_INFO(this->get_logger(), "ctrl_long_emergency node has been started");
}

void brakingSystem::CtrlLongEmergency::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    m_egoSpeed = msg->twist.twist.linear.x;
}

void brakingSystem::CtrlLongEmergency::trajectoryCallback(
    const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
    if (msg->points.empty() ) {
        m_control_msg.longitudinal.velocity = m_egoSpeed;
        return;
    }
    int point_index = std::min(static_cast<int>(msg->points.size()) - 1, static_cast<int>(25));
    float target_velocity = msg->points.at(point_index).longitudinal_velocity_mps;
    if (target_velocity < 0.5)
    {
        target_velocity = 0.0f;
    }
    //target_velocity = std::min(0.0f, target_velocity);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "target_velocity: %.3f", target_velocity);

    m_control_msg.longitudinal.velocity = target_velocity;
}

void brakingSystem::CtrlLongEmergency::run()
{
    m_control_msg.stamp = this->now();
    m_pubControl_->publish(m_control_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brakingSystem::CtrlLongEmergency>());
  rclcpp::shutdown();
  return 0;
}