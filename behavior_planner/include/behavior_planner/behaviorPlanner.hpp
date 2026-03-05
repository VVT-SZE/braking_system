#ifndef BEHAVIOR_PLANNER_HPP
#define BEHAVIOR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <crp_msgs/msg>

namespace brakingSystem
{

    class BehaviorPlanner : public rclcpp::Node
    {
    public:
        BehaviorPlanner();

    private:
        void ScenarioCallback(const crp_msgs::msg::scenario::SharedPtr msg);
        void EgoCallback(const crp_msgs::msg::ego::SharedPtr msg);

        rclcpp::Subscription<crp_msgs::msg::scenario>::SharedPtr m_subScenario_;
        rclcpp::Subscription<crp_msgs::msg::ego>::SharedPtr m_subEgo_;

        rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr m_pubScenario_;
        rclcpp::Publisher<crp_msgs::msg::TargetSpace>::SharedPtr m_pubTargetSpace_;
    };

} // namespace brakingSystem

#endif // BEHAVIOR_PLANNER_HPP