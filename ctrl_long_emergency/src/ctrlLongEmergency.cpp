#include <ctrl_long_emergency/ctrlLongEmergency.hpp>

brakingSystem::CtrlLongEmergency ::CtrlLongEmergency () : Node("ctrl_long_emergency")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_trajectory", "plan/longEmergency/trajectory");
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

    m_subTrajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        inputTopicTrajectory,
        1,
        std::bind(&CtrlLongEmergency::trajectoryCallback, this, std::placeholders::_1));

    m_pubControl_ = this->create_publisher<autoware_control_msgs::msg::Control>(
        outputTopicControl,
        1);

    
    m_control_msg.longitudinal.velocity = 0.0;
    m_control_msg.longitudinal.acceleration = 0.0;
    m_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&brakingSystem::CtrlLongEmergency::run, this));  

    RCLCPP_INFO(this->get_logger(), "ctrl_long_emergency node has been started");
}

void brakingSystem::CtrlLongEmergency::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    (void)msg;    
}

void brakingSystem::CtrlLongEmergency::trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
if (msg->points.empty()) return;

    bool stop_detected = false;

    for (const auto & point : msg->points)
    {
        if (point.longitudinal_velocity_mps <= 0.001)
        {
            stop_detected = true;
            break;
        }
    }

    if (stop_detected)
    {
        m_control_msg.longitudinal.velocity = 0.0;
        m_control_msg.longitudinal.acceleration = -5.0;
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP");
    }
    else
    {
        m_control_msg.longitudinal.velocity = msg->points[0].longitudinal_velocity_mps;
        m_control_msg.longitudinal.acceleration = 0.0; 
    }
}

void brakingSystem::CtrlLongEmergency::run()
{
    m_control_msg.stamp = this->now();
    m_pubControl_->publish(m_control_msg);

    RCLCPP_INFO(this->get_logger(), "Published control command");
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brakingSystem::CtrlLongEmergency>());
  rclcpp::shutdown();
  return 0;
}