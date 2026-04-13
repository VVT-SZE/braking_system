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
    // scenario check
    bool is_emergency = 
        (m_current_scenario_ == "LONG_EMERGENCY_AVOID" || 
         m_current_scenario_ == "LONG_EMERGENCY_IMPACT");
        
    if (is_emergency) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Obstacle detected! Object count: %zu", msg->relevant_objects.size());
    }

    // empty object risk assesment, if empty just advance
    if (msg->relevant_objects.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No relevant objects");

        std::vector<std::vector<double>> trajectory = {
            {0.0, 0.0, ego_v},
            {1.0, 0.0, ego_v}
        };

        autoware_planning_msgs::msg::Trajectory traj;
        traj.header = msg->header;

        for (const auto &p : trajectory)
        {
            autoware_planning_msgs::msg::TrajectoryPoint tp;
            tp.pose.position.x = p[0];
            tp.pose.position.y = p[1];
            tp.longitudinal_velocity_mps = p[2];
            traj.points.push_back(tp);
        }

        m_pubTrajectory_->publish(traj);
        return;
    }
            
    // getting information about the object from BP
    const auto &obj = msg->relevant_objects[0];

    double obj_x = msg->target_pose.pose.position.x;
    double obj_y = msg->target_pose.pose.position.y;

    double obj_v = obj.kinematics.initial_twist_with_covariance.twist.linear.x;
    double obj_a = obj.kinematics.initial_acceleration_with_covariance.accel.linear.x;

    if (std::abs(obj_v) < 0.001) obj_v = 0.0;
    if (std::abs(obj_a) < 0.001) obj_a = 0.0;

    RCLCPP_INFO(this->get_logger(), "Object at X: %f, Y: %f", obj_x, obj_y);


    // ego state
    double ego_v = m_current_velocity_;
    double ego_a = m_current_acceleration_; 

    // calculating real trajectory with the math from utils
    std::vector<std::vector<double>> trajectory;

    if (is_emergency)
    {
        trajectory = trajectoryCalculator->calcTrajectory(
            obj_x,
            obj_v,
            obj_a,
            ego_v,
            ego_a);


            // jerk and acceloration limit check
            double max_acc = trajectoryCalculator->getMaximumTrajectoryAcceleration(obj_x, obj_v, obj_a, ego_v, ego_a);
            double max_jerk = trajectoryCalculator->getMaximumTrajectoryJerk(obj_x, obj_v, obj_a, ego_v, ego_a);

            double ACC_LIMIT = 3.0;
            double JERK_LIMIT = 5.0;

            if (max_acc > ACC_LIMIT || max_jerk > JERK_LIMIT)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Limit exceeded! Acc: %.2f | Jerk: %.2f",
                    max_acc, max_jerk);

                // fallback
                trajectory.clear();
                trajectory.push_back({0.5, 0.0, ego_v * 0.5}); // not a hardstop but rather comfier
            }
    
    } 
    else 
    {   
        // normal advance
        trajectory = {
        {0.0, 0.0, ego_v},
        {1.0, 0.0, ego_v}
        };          
    }

    // convert to ros message
    autoware_planning_msgs::msg::Trajectory trajectory_msg;
    trajectory_msg.header = msg->header;

    for (const auto &p : trajectory)
    {
        autoware_planning_msgs::msg::TrajectoryPoint tp;

        tp.pose.position.x = p[0];
        tp.pose.position.y = p[1];
        tp.longitudinal_velocity_mps = p[2];

        trajectory_msg.points.push_back(tp);
    }

    // publish trajectory
    m_pubTrajectory_->publish(trajectory_msg);

    RCLCPP_INFO(this->get_logger(), "Published trajectory");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brakingSystem::MotionHandler>());
  rclcpp::shutdown();
  return 0;
}