#ifndef TRAJECTORY_CALCULATION_HPP
#define TRAJECTORY_CALCULATION_HPP

#include <vector>

namespace brakingSystem
{
    class TrajectoryCalculation
    {
    public:
        TrajectoryCalculation();

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
        double safety_distance;
    };

} // namespace brakingSystem

#endif // BEHAVIOR_PLANNER_HPP