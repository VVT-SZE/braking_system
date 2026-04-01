#ifndef CTRL_LONG_EMERGENCY_HPP
#define CTRL_LONG_EMERGENCY_HPP

#include <rclcpp/rclcpp.hpp>
#include <crp_msgs/msg/ego.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_control_msgs/msg/control.hpp>

namespace brakingSystem
{

class CtrlLongEmergency : public rclcpp::Node
    {
    public:
        CtrlLongEmergency();

    private:
        void egoCallback(const crp_msgs::msg::Ego::SharedPtr msg);
        void trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
        void run();

        rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr m_subEgo_;
        rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr m_subTrajectory_;

        rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr m_pubControl_;

        autoware_control_msgs::msg::Control m_control_msg;

        rclcpp::TimerBase::SharedPtr m_timer_;
    };

} // namespace CtrlLongEmergency

#endif // CTRL_LONG_EMERGENCY_HPP