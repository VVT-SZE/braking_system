#include <iostream>
#include "trajectoryCalculation.hpp"

int main() {
    brakingSystem::TrajectoryCalculation trajCalc;

    double critical_X = 20.0;
    double ego_v = 5.0;

    auto trajectory = trajCalc.calcTrajectory(critical_X, 0.0, 0.0, ego_v, 0.0);

    std::cout << "Trajectory points:\n";
    for (const auto &point : trajectory) {
        for (double val : point) {
            std::cout << val << " ";
        }
        std::cout << "\n";
    }

    std::cout << "Max Acc: " << trajCalc.getMaximumTrajectoryAcceleration(critical_X, 0.0, 0.0, ego_v, 0.0) << "\n";
    std::cout << "Max Jerk: " << trajCalc.getMaximumTrajectoryJerk(critical_X, 0.0, 0.0, ego_v, 0.0) << "\n";

    return 0;
}