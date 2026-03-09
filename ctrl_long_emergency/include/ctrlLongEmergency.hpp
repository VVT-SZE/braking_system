#ifndef CTRL_LONG_EMERGENCY_HPP
#define CTRL_LONG_EMERGENCY_HPP

#include <rclcpp/rclcpp.hpp>
#include <crp_msgs/msg>

namespace brakingSystem
{

class CtrlLongEmergency : public rclcpp::Node
{
public:
    CtrlLongEmergency();

private:
    void egoCallback(const crp_msgs::msg::ego::SharedPtr msg);
    void trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);

    rclcpp::Subscription<crp_msgs::msg::ego>::SharedPtr m_subEgo_;
    rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr m_subTrajectory_;

    rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr m_pubControl_;
}

} // namespace CtrlLongEmergency

#endif // CTRL_LONG_EMERGENCY_HPP