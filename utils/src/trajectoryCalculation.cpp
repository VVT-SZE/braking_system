#include "utils/trajectoryCalculation.hpp"
#include <vector>
#include <stdexcept>
#include <cmath>
#include <utility>

namespace brakingSystem
{

    TrajectoryCalculation::TrajectoryCalculation(double safety_distance)
    {
        safety_distance_ = safety_distance;
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
        double stop_x = criticalObject_X - safety_distance_; // dont stop at obj stop at the safety_dist

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
            trajectory.push_back({0.0, 0.0, 0.0});
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

    std::vector<double> calculateCoefficients(
                double criticalObject_X, 
                double criticalObject_vX, 
                double criticalObject_aX, 
                double egoObject_vX, 
                double egoObject_aX 
            ) {
        double x_tar = criticalObject_X - safety_distance_;
        
        double dv = egoObject_vX - criticalObject_vX;
        
        // Safety check: if we are already going slower than the target, or dv is 0,
        // we don't need a braking trajectory to avoid collision. 
        if (dv <= 0.0) {
            return {egoObject_vX, egoObject_aX, 0.0, 0.0}; 
        }
        
        double v_mean = dv / 2.0;
        
        // Prevent division by zero just in case
        if (v_mean <= 0.001) {
            v_mean = 0.001; 
        }
        
        double t_c = x_tar / v_mean;

        std::vector<std::vector<double>> A = {
            {1.0, 0.0, 0.0, 0.0},
            {0.0, 1.0, 0.0, 0.0},
            {1.0, t_c, t_c * t_c, t_c * t_c * t_c},
            {0.0, 1.0, 2.0 * t_c, 3.0 * t_c * t_c}
        };

        std::vector<std::vector<double>> b = {
            {egoObject_vX},
            {egoObject_aX},
            {criticalObject_vX},
            {criticalObject_aX}
        };

        std::vector<std::vector<double>> c_matrix = mxMul(inv(A), b);

        std::vector<double> coefficients = {
            c_matrix[0][0], // c0
            c_matrix[1][0], // c1
            c_matrix[2][0], // c2
            c_matrix[3][0]  // c3
        };

        return coefficients;
    }


    std::vector<std::vector<double>> TrajectoryCalculation::inv(std::vector<std::vector<double>> mat) {
        int n = mat.size();
        
        if (n == 0 || mat[0].size() != n) {
            throw std::invalid_argument("Matrix must be square and non-empty.");
        }

        std::vector<std::vector<double>> aug(n, std::vector<double>(2 * n, 0.0));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                aug[i][j] = mat[i][j];
            }
            aug[i][i + n] = 1.0; 
        }

        for (int i = 0; i < n; ++i) {
            double pivot = aug[i][i];
            int pivot_row = i;
            for (int k = i + 1; k < n; ++k) {
                if (std::abs(aug[k][i]) > std::abs(pivot)) {
                    pivot = aug[k][i];
                    pivot_row = k;
                }
            }

            if (std::abs(pivot) < 1e-9) {
                throw std::runtime_error("Matrix is singular and cannot be inverted.");
            }

            if (pivot_row != i) {
                std::swap(aug[i], aug[pivot_row]);
            }

            for (int j = 0; j < 2 * n; ++j) {
                aug[i][j] /= pivot;
            }

            for (int k = 0; k < n; ++k) {
                if (k != i) {
                    double factor = aug[k][i];
                    for (int j = 0; j < 2 * n; ++j) {
                        aug[k][j] -= factor * aug[i][j];
                    }
                }
            }
        }

        std::vector<std::vector<double>> inverse(n, std::vector<double>(n, 0.0));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                inverse[i][j] = aug[i][j + n];
            }
        }

        return inverse;
    }

    std::vector<std::vector<double>> TrajectoryCalculation::mxMul(std::vector<std::vector<double>> mxA, std::vector<std::vector<double>> mxB) {
        if (mxA.empty() || mxB.empty() || mxA[0].empty() || mxB[0].empty()) {
            throw std::invalid_argument("Matrices cannot be empty.");
        }

        int rowsA = mxA.size();
        int colsA = mxA[0].size();
        int rowsB = mxB.size();
        int colsB = mxB[0].size();

        if (colsA != rowsB) {
            throw std::invalid_argument("Dimension mismatch: Columns of A must equal Rows of B.");
        }

        std::vector<std::vector<double>> result(rowsA, std::vector<double>(colsB, 0.0));

        for (int i = 0; i < rowsA; ++i) {
            for (int j = 0; j < colsB; ++j) {
                for (int k = 0; k < colsA; ++k) {
                    result[i][j] += mxA[i][k] * mxB[k][j];
                }
            }
        }

        return result;
    }

}