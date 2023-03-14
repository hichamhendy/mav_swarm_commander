#pragma once

#include "esdf_map/signed_distance_field.h" // remove later
#include "mav_swarm_commander/state_space_ODE.h"

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
        residual[0] = 0.5 * pow(dist.norm(),2) * T(weight_);

        return true;
    }

    private:
    const double weight_;
};


 
class FG_eval 
{   
    public:
        const size_t N_ = 6;
        const double dt_ = 0.05; // connect with ros
        size_t x_start, y_start, z_start, x_dot_start, y_dot_start, z_dot_start, roll_start, pitch_start, roll_command_start, pitch_command_start, thrust_command_start;
        const Eigen::Vector3d p1_ref_;
        const Eigen::Vector3d p2_ref_;

        typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
        // Coefficients of the fitted polynomial.
        FG_eval(const size_t N, const double dt, const Eigen::Vector3d p1_ref, const Eigen::Vector3d p2_ref):
            N_(N), dt_(dt), p1_ref_(p1_ref), p2_ref_(p2_ref)    
        {
            x_start = 0;
            y_start = x_start + N_;
            z_start = y_start + N_;
            x_dot_start = z_start + N_;
            y_dot_start = x_dot_start + N_;
            z_dot_start = y_dot_start + N_;
            roll_start =  z_dot_start + N_;
            pitch_start = roll_start + N_;
            roll_command_start = pitch_start + N_;
            pitch_command_start = roll_command_start + N_ - 1;
            thrust_command_start = pitch_command_start + N_ - 1;
        }

        
        // `fg` is a vector containing the cost and constraints.
        // `vars` is a vector containing the variable values (state & actuators).
        void operator()(ADvector& fg, const ADvector& vars) 
        {
            assert( vars.size()  == (N_ * 8 + (N_ - 1) * 3) );
            // The cost is stored is the first element of `fg`.
            // Any additions to the cost should be added to `fg[0]`.
            fg[0] = 0;
            const Eigen::Vector3d vector_direction = (p2_ref_ - p1_ref_).normalized();
            const double segment_length = (p2_ref_ - p1_ref_).norm();
            const double step_length = segment_length / N_;

            // Reference State Cost
            for (size_t t = 0; t < N_; ++t)
            {
                const Eigen::Vector3d inspection_pos = p1_ref_ + vector_direction * step_length * (t); // should be (t) or (t + 1) ????

                fg[0] += CppAD::pow(vars[x_start + t] - inspection_pos.x(), 2);
                fg[0] += CppAD::pow(vars[y_start + t] - inspection_pos.y(), 2);
                fg[0] += CppAD::pow(vars[z_start + t] - inspection_pos.z(), 2);
            }
 
            // Minimize the use of actuators.
            for (size_t t = 0; t < N_ - 1; ++t) 
            {
                fg[0] += CppAD::pow(vars[roll_command_start + t], 2);
                fg[0] += CppAD::pow(vars[pitch_command_start + t], 2);
                fg[0] += CppAD::pow(vars[thrust_command_start + t], 2); /// 
            }

            // Minimize the value gap between sequential actuations to avoid overexcertion which can't be anyway implemented by PX4.
            for (size_t t = 0; t < N_ - 2; ++t) 
            {
                fg[0] += CppAD::pow(vars[roll_command_start + t + 1] - vars[roll_command_start + t], 2);
                fg[0] += CppAD::pow(vars[pitch_command_start + t + 1] - vars[pitch_command_start + t], 2);
                fg[0] += CppAD::pow(vars[thrust_command_start + t + 1] - vars[thrust_command_start + t], 2);
            }
    
             
            // Setup Constraints
            // NOTE: In this section, setup the model constraints.

            // Initial constraints
            // add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
            // This bumps up the position of all the other values.
            fg[1 + x_start] = vars[x_start];
            fg[1 + y_start] = vars[y_start];
            fg[1 + z_start] = vars[z_start];
            fg[1 + x_dot_start] = vars[x_dot_start];
            fg[1 + y_dot_start] = vars[y_dot_start];
            fg[1 + z_dot_start] = vars[z_dot_start];
            fg[1 + roll_start] = vars[roll_start];
            fg[1 + pitch_start] = vars[pitch_start];
            
            // The rest of the constraints
            for (int t = 1; t < N_; ++t) // Note that we start the loop at t=1, because the values at t=0 are set to our initial state - those values are not calculated by the solver.
            {
                // The state at time t+1 
                CppAD::AD<double> x1 = vars[x_start + t];
                CppAD::AD<double> y1 = vars[y_start + t];
                CppAD::AD<double> z1 = vars[z_start + t];
                CppAD::AD<double> x_dot1 = vars[x_dot_start + t];
                CppAD::AD<double> y_dot1 = vars[y_dot_start + t];
                CppAD::AD<double> z_dot1 = vars[z_dot_start + t];
                CppAD::AD<double> roll1 = vars[roll_start + t];
                CppAD::AD<double> pitch1 = vars[pitch_start + t];

                // The state at time t
                CppAD::AD<double> x0 = vars[x_start + t - 1];
                CppAD::AD<double> y0 = vars[y_start + t - 1];
                CppAD::AD<double> z0 = vars[z_start + t - 1];
                CppAD::AD<double> x_dot0 = vars[x_dot_start + t - 1];
                CppAD::AD<double> y_dot0 = vars[y_dot_start + t - 1];
                CppAD::AD<double> z_dot0 = vars[z_dot_start + t - 1];
                CppAD::AD<double> roll0 = vars[roll_start + t - 1];
                CppAD::AD<double> pitch0 = vars[pitch_start + t - 1];

                // Only consider the actuation at time t
                CppAD::AD<double> roll_command0 = vars[roll_command_start + t - 1];
                CppAD::AD<double> pitch_command0 = vars[pitch_command_start + t - 1];
                CppAD::AD<double> thrust_command0 = vars[thrust_command_start + t - 1];

                fg[1 + x_start + t] = x1 - (x0 + x_dot0 * dt_);
                fg[1 + y_start + t] = y1 - (y0 + y_dot0 * dt_);
                fg[1 + z_start + t] = z1 - (z0 + z_dot0 * dt_);
                fg[1 + x_dot_start + t] = x_dot1 - (x_dot0 + (-0.01 * x_dot0 + 9.81 * roll0) * dt_);
                fg[1 + y_dot_start + t] = y_dot1 - (y_dot0 + (-0.01 * y_dot0 - 9.81 * pitch0) * dt_);
                fg[1 + z_dot_start + t] = z_dot1 - (z_dot0 + (thrust_command0) * dt_);
                fg[1 + roll_start + t] = roll1 - (-roll0 + roll_command0);
                fg[1 + pitch_start + t] = pitch1 - (-pitch0 + pitch_command0);               
            }
        }
};


// class ocp
// {

//     public:
//         /**
//          * @brief Construct a new ocp object
//          * 
//          */
//         ocp()N_(N), dt_(dt), p1_ref_(p1_ref), p2_ref_(p2_ref)  
//         {

//         }

//         /**
//          * @brief Destroy the ocp object
//          * 
//          */
//         ~ocp()
//         {
            
//         }

//         double costFunction(const std::vector<double>& vars, std::vector<double>& grad,
//                                         void* func_data)
//         {

//             double cost;
//             const Eigen::Vector3d vector_direction = (p2_ref_ - p1_ref_).normalized();
//             const double segment_length = (p2_ref_ - p1_ref_).norm();
//             const double step_length = segment_length / N_;

//             if (!grad.empty()) 
//             {
//                 // grad[0] = 0.0;
//                 // grad[1] = 0.5 / sqrt(x[1]);
//             }

//             // Reference State Cost
//             for (size_t t = 0; t < N_; ++t)
//             {
//                 const Eigen::Vector3d inspection_pos = p1_ref_ + vector_direction * step_length * (t); // should be (t) or (t + 1) ????

//                 cost += pow(vars[x_start + t] - inspection_pos.x(), 2);
//                 cost += pow(vars[y_start + t] - inspection_pos.y(), 2);
//                 cost += pow(vars[z_start + t] - inspection_pos.z(), 2);
//             }

//                 // Minimize the use of actuators.
//             for (size_t t = 0; t < N_ - 1; ++t) 
//             {
//                 cost += pow(vars[roll_command_start + t], 2);
//                 cost += pow(vars[pitch_command_start + t], 2);
//                 cost += pow(vars[thrust_command_start + t], 2); /// 
//             }

//                 // Minimize the value gap between sequential actuations to avoid overexcertion which can't be anyway implemented by PX4.
//             for (size_t t = 0; t < N_ - 2; ++t) 
//             {
//                 cost += pow(vars[roll_command_start + t + 1] - vars[roll_command_start + t], 2);
//                 cost += pow(vars[pitch_command_start + t + 1] - vars[pitch_command_start + t], 2);
//                 cost += pow(vars[thrust_command_start + t + 1] - vars[thrust_command_start + t], 2);
//             }

//             return cost;
//         }


//         double myconstraint(unsigned n, const double *x, double *grad, void *data)
//         {
            
//         }

//     private:
//         private:
//         const size_t N_ = 6;
//         const double dt_ = 0.05; // connect with ros
//         const size_t x_start = 0;
//         const size_t y_start = x_start + N_;
//         const size_t z_start = y_start + N_;
//         const size_t x_dot_start = z_start + N_;
//         const size_t y_dot_start = x_dot_start + N_;
//         const size_t z_dot_start = y_dot_start + N_;
//         const size_t roll_start =  z_dot_start + N_;
//         const size_t pitch_start = roll_start + N_;
//         const size_t roll_command_start = pitch_start + N_;
//         const size_t pitch_command_start = roll_command_start + N_ - 1;
//         const size_t thrust_command_start = pitch_command_start + N_ - 1;
//         const Eigen::Vector3d p1_ref_;
//         const Eigen::Vector3d p2_ref_;
// };
