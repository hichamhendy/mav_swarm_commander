#pragma once

#include "esdf_map/signed_distance_field.h" // remove later

class DistanceToObstacleFunctor
{
public:
    /**
     * @brief 
     * ceres::CENTRAL -> Finite Differencing Scheme ->  more accurate than FORWARD at the cost of twice as many function evaluations than forward difference
     * ceres::DYNAMIC -> Indicate dynamic number of residuals, since that is dependent on num_steps that varies pased on the length beween points
     * 3 - Dimension of x
     * 3 - Dimension of y
     * 
     */
    using CostFunction = ceres::NumericDiffCostFunction<DistanceToObstacleFunctor, ceres::CENTRAL, ceres::DYNAMIC, 3, 3>; // http://ceres-solver.org/nnls_modeling.html#numericdiffcostfunction

    /**
     * @brief Construct a new Distance To Obstacle Functor object
     * 
     * @param map 
     * @param num_steps 
     * @param min_dist_soft 
     * @param min_dist_hard 
     * @param weight_soft_obstacle 
     * @param weight_hard_obstacle 
     */
    DistanceToObstacleFunctor(const VoxelGridMap& map, const std::size_t num_steps, const double min_dist_soft,
                                const double min_dist_hard, const double weight_soft_obstacle, const double weight_hard_obstacle)
            :ob_map_(map), num_steps_(num_steps), min_dist_soft_(min_dist_soft), min_dist_hard_(min_dist_hard),
            weight_soft_obstacle_(weight_soft_obstacle), weight_hard_obstacle_(weight_hard_obstacle)
    {
    }

    /**
     * @brief overloading the () operator to compute the residuals
     * more info about constructions // http://ceres-solver.org/nnls_modeling.html#numericdiffcostfunction
     * 
     * @param raw_pos1 
     * @param raw_pos2 
     * @param residual -> The functor writes the computed value here. Shall NOT be constant
     * @return true 
     * @return false 
     */
    bool operator()(const double* const raw_pos1, const double* const raw_pos2, double* residual) const
    {
        const Eigen::Vector3d p1{raw_pos1[0], raw_pos1[1], raw_pos1[2]};
        const Eigen::Vector3d p2{raw_pos2[0], raw_pos2[1], raw_pos2[2]};

        const Eigen::Vector3d vector_direction = (p2 - p1).normalized();
        const double segment_length = (p2 - p1).norm();

        const double step_length = segment_length / num_steps_;

        for (int i = 0; i < num_steps_; i++)
        {
            const Eigen::Vector3d inspection_pos = p1 + vector_direction * step_length * (i + 1);
            double dist = 0.0;

            if (!std::isfinite(inspection_pos.x()) || !std::isfinite(inspection_pos.y()) ||
                !std::isfinite(inspection_pos.z())) // Catch accessed unallocated block crash
            {
                dist = 0.0;
            }
            else
                dist = ob_map_.getDistance(inspection_pos);

            if (dist < min_dist_hard_) // In hard boundary => very fast rising cost
            {
                residual[i] = ((min_dist_hard_ - dist) * weight_hard_obstacle_) + ((min_dist_soft_ - dist) * weight_soft_obstacle_); // to be redesigned  based on thesis paper
            }
            // In soft boundary => slowly rising cost
            else if (dist < min_dist_soft_)
            {
                residual[i] = (min_dist_soft_ - dist) * weight_soft_obstacle_;
            }
            else // Far enough away => No cost
            {
                residual[i] = 0;
            }
            }

            return true;
    }

private:
    const VoxelGridMap& ob_map_;
    const double min_dist_soft_;
    const double min_dist_hard_;
    const double weight_soft_obstacle_;
    const double weight_hard_obstacle_;
    const std::size_t num_steps_;
};



class TrackingGlobalPlannerFunctor
{
public:
    /**
     * @brief 
     * 1 -> kNumResiduals, which Indicate 1 dimensional of residuals
     * 3 - Dimension of x
     * 3 - Dimension of y
     * More Infoon http://ceres-solver.org/nnls_modeling.html#autodiffcostfunction
     */
    using CostFunction = ceres::AutoDiffCostFunction<TrackingGlobalPlannerFunctor, 1, 3, 3>;

    TrackingGlobalPlannerFunctor(const double weight) : weight_(weight) {}

    /**
     * @brief 
     * 
     * @tparam T 
     * @param raw_pos1 
     * @param raw_pos2 
     * @param residual 
     * @return true 
     * @return false 
     */
    template <typename T> 
    bool operator()(const T* const raw_pos1, const T* const raw_pos2, T* residual) const
    {
        const Eigen::Matrix<T, 3, 1> p1{raw_pos1[0], raw_pos1[1], raw_pos1[2]};
        const Eigen::Matrix<T, 3, 1> p2{raw_pos2[0], raw_pos2[1], raw_pos2[2]};

        const Eigen::Matrix<T, 3, 1> dist = p1 - p2;
        residual[0] = dist.norm() * T(weight_);

        return true;
    }

    private:
    const double weight_;
};
