#include "trajectoryCalculation.hpp"

namespace brakingSystem
{

    TrajectoryCalculation::TrajectoryCalculation()
    {
        safety_distance = 5.0; // TODO: get from param
    }


    std::vector<std::vector<double>> TrajectoryCalculation::calcTrajectory(
    double criticalObject_X,
    double /*criticalObject_vX*/,
    double /*criticalObject_aX*/,
    double /*egoObject_vX*/,
    double /*egoObject_aX*/)
    {
        std::vector<std::vector<double>> trajectory;
        
        double step = 1.0; // point at every meter
        double stop_x = criticalObject_X - safety_distance; // dont stop at obj stop at the safety_dist

        if (stop_x < 0.0)
        {
            stop_x = 0.0;
        }

        for (double x=0.0; x <= stop_x; x+=step)
        {
            trajectory.push_back({
                x,    
                0.0,
                0.0
            });
        }

        if (trajectory.empty())
        {
            trajectory.push_back({0.0, 0.0});
        }
        
        return trajectory; 
    }

    double TrajectoryCalculation::getMaximumTrajectoryAcceleration(
        double /*criticalObject_X*/,
        double /*criticalObject_vX*/,
        double /*criticalObject_aX*/,
        double /*egoObject_vX*/,
        double /*egoObject_aX*/)
    {
        // TODO
        return 0.0;
    }

    double TrajectoryCalculation::getMaximumTrajectoryJerk(
        double /*criticalObject_X*/,
        double /*criticalObject_vX*/,
        double /*criticalObject_aX*/,
        double /*egoObject_vX*/,
        double /*egoObject_aX*/)
    {
        // TODO
        return 0.0;
    }
}