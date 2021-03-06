// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once


#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>
#include <assert.h>

#include <vector>
#define DCHECK_GT(a,b) assert((a)>(b))
#define DCHECK_EQ(a,b) assert((a)==(b))

#ifdef _WIN32
typedef __int64 qp_int64;
#else
typedef long long qp_int64;
#endif //_WIN32


using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
#include "qpOASES.hpp"
#include "qpOASES/Types.hpp"

using qpOASES::QProblem;
  
typedef Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic,
                      Eigen::RowMajor>
    RowMajorMatrixXd;
    
constexpr int k3Dim = 3;
constexpr double kGravity = 9.8;
constexpr double kMaxScale = 10;
constexpr double kMinScale = 0.1;

#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "ctrlc.h"
#include "osqp.h"


enum QPSolverName
{
  OSQP, QPOASES
};

// Converts the roll pitchh yaw angle vector to the corresponding rotation
// matrix.
Eigen::Matrix3d ConvertRpyToRot(const Eigen::Vector3d& rpy);

// Converts a vector to the skew symmetric matrix form. For an input vector
// [a, b, c], the output matrix would be:
//   [ 0, -c,  b]
//   [ c,  0, -a]
//   [-b,  a,  0]
Eigen::Matrix3d ConvertToSkewSymmetric(const Eigen::Vector3d& vec);

// The CoM dynamics can be written as:
//   X_dot = A X + B u
// where X is the 13-dimensional state vector (r, p, y, x, y, z, r_dot, p_dot,
// y_dot, vx, vy, vz, -g) constructed from the CoM roll/pitch/yaw/position, and
// their first order derivatives. 'g' is the gravity constant. This API
// constructs the A matrix in the formula. Check the MIT paper for details of
// the formulation.
void CalculateAMat(const Eigen::Vector3d& rpy, Eigen::MatrixXd* a_mat_ptr);

// Constructs the B matrix in the linearized CoM dynamics equation. See the
// documentation for 'CalculateAMat' for details of the symbol.
void CalculateBMat(double inv_mass, const Eigen::Matrix3d& inv_inertia,
    const Eigen::MatrixXd& foot_positions,
    Eigen::MatrixXd* b_mat_ptr);

// Calculates the discretized space-time dynamics. Given the dynamics equation:
//   X_dot = A X + B u
// and a timestep dt, we can estimate the snapshot of the state at t + dt by:
//   X[t + dt] = exp([A, B]dt) [X, u] = A_exp X + B_exp u
void CalculateExponentials(const Eigen::MatrixXd& a_mat,
    const Eigen::MatrixXd& b_mat, double timestep,
    Eigen::MatrixXd* ab_mat_ptr,
    Eigen::MatrixXd* a_exp_ptr,
    Eigen::MatrixXd* b_exp_ptr);

// Calculates the dense QP formulation of the discretized space time dynamics.
// Given:
//   X_k+1 = A_exp X_k + B_exp u_k
// We can unroll the dynamics in time using a forward pass:
//   [X_1, X_2,..., X_k+1] = A_qp X_0 + B_qp [u_0, u_1,..., u_k]
void CalculateQpMats(const Eigen::MatrixXd& a_exp, const Eigen::MatrixXd& b_exp,
    int horizon, Eigen::MatrixXd* a_qp_ptr,
    Eigen::MatrixXd* b_qp_ptr);

void UpdateConstraintsMatrix(std::vector<double>& friction_coeff,
    int horizon, int num_legs,
    Eigen::MatrixXd* constraint_ptr);

void CalculateConstraintBounds(const Eigen::MatrixXd& contact_state, double fz_max,
    double fz_min, double friction_coeff, int horizon,
    Eigen::VectorXd* constraint_lb_ptr,
    Eigen::VectorXd* constraint_ub_ptr);

double EstimateCoMHeightSimple(const Eigen::MatrixXd& foot_positions_world,
    const std::vector<bool> foot_contact_states);

// The MIT convex mpc implementation as described in this paper:
//   https://ieeexplore.ieee.org/document/8594448/
// Computes the optimal feet contact forces given a desired center of mass
// trajectory and gait pattern.
class ConvexMpc {
public:
    static constexpr int kStateDim =
        13;  // 6 dof pose + 6 dof velocity + 1 gravity.

    // For each foot contact force we use 4-dim cone approximation + 1 for z.
    static constexpr int kConstraintDim = 5;

    ConvexMpc();

    ConvexMpc(double mass, const std::vector<double>& inertia, int num_legs,
        int planning_horizon, double timestep,
        const std::vector<double>& qp_weights, double alpha = 1e-5, 
          QPSolverName qp_solver_name=QPOASES);

    virtual ~ConvexMpc()
    {
        osqp_cleanup(workspace_);
    }
    // If not explicitly specified, we assume the quantities are measured in a
    // world frame. Usually we choose the yaw-aligned horizontal frame i.e. an
    // instanteneous world frame at the time of planning with its origin at CoM
    // and z axis aligned with gravity. The yaw-alignment means that the CoM
    // rotation measured in this frame has zero yaw component. Caveat: We expect
    // the input euler angle roll_pitch_yaw to be in ZYX format, i.e. the rotation
    // order is X -> Y -> Z, with respect to the extrinsic (fixed) coordinate
    // frame. In the intrinsic (body-attached) frame the rotation order is Z -> Y'
    // -> X".
    std::vector<double> ComputeContactForces(
        std::vector<double> com_position,
        std::vector<double> com_velocity,
        std::vector<double> com_roll_pitch_yaw,
        std::vector<double> com_angular_velocity,
        std::vector<int> foot_contact_states,
        std::vector<double> foot_positions_body_frame,
        std::vector<double> foot_friction_coeffs,
        std::vector<double> desired_com_position,
        std::vector<double> desired_com_velocity,
        std::vector<double> desired_com_roll_pitch_yaw,
        std::vector<double> desired_com_angular_velocity);

    // Reset the solver so that for the next optimization run the solver is
    // re-initialized.
    void ResetSolver();

private:
    const double mass_;
    const double inv_mass_;
    const Eigen::Matrix3d inertia_;
    const Eigen::Matrix3d inv_inertia_;
    const int num_legs_;
    const int planning_horizon_;
    const double timestep_;
    QPSolverName qp_solver_name_;

    // 13 * horizon diagonal matrix.
    const Eigen::MatrixXd qp_weights_;

    // 13 x 13 diagonal matrix.
    const Eigen::MatrixXd qp_weights_single_;

    // num_legs * 3 * horizon diagonal matrix.
    const Eigen::MatrixXd alpha_;
    const Eigen::MatrixXd alpha_single_;
    const int action_dim_;

    // The following matrices will be updated for every call. However, their sizes
    // can be determined at class initialization time.
    Eigen::VectorXd state_;                 // 13
    Eigen::VectorXd desired_states_;        // 13 * horizon
    Eigen::MatrixXd contact_states_;        // horizon x num_legs
    Eigen::MatrixXd foot_positions_base_;   // num_legs x 3
    Eigen::MatrixXd foot_positions_world_;  // num_legs x 3
    Eigen::VectorXd foot_friction_coeff_;   // num_legs
    Eigen::Matrix3d rotation_;
    Eigen::Matrix3d inertia_world_;    // rotation x inertia x rotation_transpose
    Eigen::MatrixXd a_mat_;            // 13 x 13
    Eigen::MatrixXd b_mat_;            // 13 x (num_legs * 3)
    Eigen::MatrixXd ab_concatenated_;  // 13 + num_legs * 3 x 13 + num_legs * 3
    Eigen::MatrixXd a_exp_;            // same dimension as a_mat_
    Eigen::MatrixXd b_exp_;            // same dimension as b_mat_

    // Contains all the power mats of a_exp_. Consider Eigen::SparseMatrix.
    Eigen::MatrixXd a_qp_;  // 13 * horizon x 13
    Eigen::MatrixXd b_qp_;  // 13 * horizon x num_legs * 3 * horizon sparse
    Eigen::MatrixXd b_qp_transpose_;
    Eigen::MatrixXd p_mat_;  // num_legs * 3 * horizon x num_legs * 3 * horizon
    Eigen::VectorXd q_vec_;  // num_legs * 3 * horizon vector

    // Auxiliary containing A^n*B, with n in [0, num_legs * 3)
    Eigen::MatrixXd anb_aux_;  // 13 * horizon x (num_legs * 3)

    // Contains the constraint matrix and bounds.
    Eigen::MatrixXd
        constraint_;  // 5 * num_legs * horizon x 3 * num_legs * horizon
    Eigen::VectorXd constraint_lb_;  // 5 * num_legs * horizon
    Eigen::VectorXd constraint_ub_;  // 5 * num_legs * horizon

    std::vector<double> qp_solution_;
    
    ::OSQPWorkspace* workspace_;
    // Whether optimizing for the first step
    bool initial_run_;
};

constexpr int ConvexMpc::kStateDim;