#ifndef TRAJECTORY_CALCULATION_HPP
#define TRAJECTORY_CALCULATION_HPP

#include <vector>

namespace brakingSystem
{
    class TrajectoryCalculation
    {
    public:
        TrajectoryCalculation(double safety_distance = 5.0);

        std::vector<std::vector<double>> calcTrajectory(
            double criticalObject_X,
            double criticalObject_vX,
            double criticalObject_aX,
            double egoObject_vX,
            double egoObject_aX);

        double getMaximumTrajectoryAcceleration(
            double criticalObject_X,
            double criticalObject_vX,
            double criticalObject_aX,
            double egoObject_vX,
            double egoObject_aX);

        double getMaximumTrajectoryJerk(
            double criticalObject_X,
            double criticalObject_vX,
            double criticalObject_aX,
            double egoObject_vX,
            double egoObject_aX);
            
    private:
        double safety_distance_;
        double max_velocity;
        std::vector<std::vector<double>> inv(std::vector<std::vector<double>> mat);
        std::vector<std::vector<double>> mxMul(std::vector<std::vector<double>> mxA, std::vector<std::vector<double>> mxB);
        std::vector<double> calculateCoefficients(
            double criticalObject_X,
            double criticalObject_vX,
            double criticalObject_aX,
            double egoObject_vX,
            double egoObject_aX
        );
    };

} // namespace brakingSystem

#endif // BEHAVIOR_PLANNER_HPP