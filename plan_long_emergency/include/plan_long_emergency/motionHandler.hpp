#ifndef MOTION_HANDLER_HPP
#define MOTION_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <crp_msgs/msg/ego.hpp>
#include <crp_msgs/msg/target_space.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <utils/trajectoryCalculation.hpp>

namespace brakingSystem
{

    class MotionHandler : public rclcpp::Node
    {
    public:
        MotionHandler();

    private:
        void scenarioCallback(const tier4_planning_msgs::msg::Scenario::SharedPtr msg);
        void egoCallback(const crp_msgs::msg::Ego::SharedPtr msg);
        void targetSpaceCallback(const crp_msgs::msg::TargetSpace::SharedPtr msg);

        double safety_distance; 
        brakingSystem::TrajectoryCalculation* trajectoryCalculator;

        rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr m_subEgo_;
        rclcpp::Subscription<crp_msgs::msg::TargetSpace>::SharedPtr m_subTargetSpace_;
        rclcpp::Subscription<tier4_planning_msgs::msg::Scenario>::SharedPtr m_subScenario_;

        rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr m_pubTrajectory_;

        std::string m_current_scenario_;
        double m_current_velocity_;
        double m_current_acceleration_;
    };

} // namespace brakingSystem

#endif // MOTION_HANDLER_HPP