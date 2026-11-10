#ifndef BEHAVIOR_PLANNER_HPP
#define BEHAVIOR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <crp_msgs/msg/scenario.hpp>
#include <crp_msgs/msg/ego.hpp>
#include <crp_msgs/msg/target_space.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <utils/trajectoryCalculation.hpp>
#include <memory>

namespace brakingSystem
{
    class BehaviorPlanner : public rclcpp::Node
    {
    public:
        BehaviorPlanner();

    private:
        void scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg);
        void egoCallback(const crp_msgs::msg::Ego::SharedPtr msg);
        void run();

        rclcpp::Subscription<crp_msgs::msg::Scenario>::SharedPtr m_subScenario_;
        rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr m_subEgo_;

        rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr m_pubScenario_;
        rclcpp::Publisher<crp_msgs::msg::TargetSpace>::SharedPtr m_pubTargetSpace_;

        rclcpp::TimerBase::SharedPtr m_timer_;

        std::vector<autoware_perception_msgs::msg::PredictedObject> m_critical_objects_;

        // Holds the last non-empty critical_objects snapshot so a single missed
        // detection (perception flicker) doesn't immediately snap the scenario
        // back to NO_ACTION - see object_loss_grace_period.
        std::vector<autoware_perception_msgs::msg::PredictedObject> m_last_critical_objects_;
        rclcpp::Time m_last_critical_objects_time_;
        double m_object_loss_grace_period_{0.3};

        // Store the latest ego pose and heading
        geometry_msgs::msg::Pose m_ego_pose_;
        geometry_msgs::msg::Twist m_ego_twist_;
        geometry_msgs::msg::Accel m_ego_accel_;
        float m_ego_heading_;
        double m_critical_distance_{50.0};
        double m_safety_distance_{5.0};
        double m_warning_threshold_{1.5};
        double m_emergency_threshold_{2.5};
        std::string m_highest_strategy_ = "NO_ACTION";

        std::unique_ptr<TrajectoryCalculation> m_trajectory_calculator_;
    };

} // namespace brakingSystem

#endif // BEHAVIOR_PLANNER_HPP