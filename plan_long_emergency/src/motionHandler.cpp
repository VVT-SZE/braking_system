#include <plan_long_emergency/motionHandler.hpp>

brakingSystem::MotionHandler::MotionHandler() : Node("plan_long_emergency")
{
    this->declare_parameter<std::string>("input_topic_ego", "ego");
    this->declare_parameter<std::string>("input_topic_scenario", "plan/strategy");
    this->declare_parameter<std::string>("input_topic_target_space", "plan/target_space");
    this->declare_parameter<std::string>("output_topic_trajectory", "plan/longEmergency/trajectory");
    this->declare_parameter<double>("safety_distance", 4.0);

    std::string inputTopicEgo, inputTopicScenario, inputTopicTargetSpace, outputTopicTrajectory;
    this->get_parameter("input_topic_ego", inputTopicEgo);
    this->get_parameter("input_topic_scenario", inputTopicScenario);
    this->get_parameter("input_topic_target_space", inputTopicTargetSpace);
    this->get_parameter("output_topic_trajectory", outputTopicTrajectory);
    this->get_parameter("safety_distance", safety_distance)

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
    // safety check
    if (msg->relevant_objects.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No relevant objects");
        return;
    }
    
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

    struct trajectoryPoint
    {
        double x; 
        double y;
        double v;
    };

    std::vector<trajectoryPoint> trajectory;
    
    double step = 1.0; // point at every meter
    double stop_x = obj_x - safety_distance; // dont stop at obj stop at the safety_dist

    if (stop_x < 0.0)
    {
        stop_x = 0.0;
    }

    for (double x=0.0; x <= stop_x; x+=step)
    {
        trajectory.push_back({
            x,
            0.0,    
            0.0 // break
        });
    }

    if (trajectory.empty())
    {
        trajectory.push_back({0.0, 0.0, 0.0});
    }

    // convert to ros message
    autoware_planning_msgs::msg::Trajectory trajectory_msg;
    trajectory_msg.header = msg->header;

    for (const auto &p : trajectory)
    {
        autoware_planning_msgs::msg::TrajectoryPoint tp;

        tp.pose.position.x = p.x;
        tp.pose.position.y = p.y;
        tp.longitudinal_velocity_mps = p.v;

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