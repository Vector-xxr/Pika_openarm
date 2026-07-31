// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <string.h>
#include <unistd.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>
#include <vector>
/*
 * Compute gravity and inertia compensation using Orocos
 * Kinematics and Dynamics Library (KDL).
 */
class Dynamics {
private:
    std::shared_ptr<urdf::ModelInterface> urdf_model_interface;

    std::string urdf_path;
    std::string start_link;
    std::string end_link;

    KDL::JntSpaceInertiaMatrix inertia_matrix;
    KDL::JntArray q;
    KDL::JntArray q_d;
    KDL::JntArray coriolis_forces;
    KDL::JntArray gravity_forces;

    KDL::JntArray biasangle;

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    std::unique_ptr<KDL::ChainDynParam> solver;
    std::vector<double> q_min_;
    std::vector<double> q_max_;
    bool joint_limits_valid_ = false;

public:
    Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
    ~Dynamics();

    bool Init();
    void GetGravity(const double *motor_position, double *gravity);
    void GetCoriolis(const double *motor_position, const double *motor_velocity, double *coriolis);
    void GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag);

    void GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian);

    void GetNullSpace(const double *motor_positon, Eigen::MatrixXd &nullspace);

    void GetNullSpaceTauSpace(const double *motor_position, Eigen::MatrixXd &nullspace_T);

    void GetEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);

    void GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);

    // FK to a specific chain segment (0-based). Used for shoulder/elbow anchors.
    bool GetSegmentCordinate(const double *motor_position, int segment_index, Eigen::Matrix3d &R,
                             Eigen::Vector3d &p);

    size_t GetDof() const { return kdl_chain.getNrOfJoints(); }

    size_t GetNrOfSegments() const { return kdl_chain.getNrOfSegments(); }

    bool GetJointPositionLimits(std::vector<double> *q_min, std::vector<double> *q_max) const;
    void ClampJointPositions(double *q_inout) const;

    double ComputeMinSingularValue(const double *motor_position);

    // Weighted velocity-level DLS: q_dot = W J^T (J W J^T + λ² I)^{-1} v
    bool SolveDlsIk(const double *motor_position, const Eigen::VectorXd &v_cart, double lambda,
                    Eigen::VectorXd *dq_out, double *sigma_min_out = nullptr,
                    const std::vector<double> *joint_mobility_weights = nullptr);

    // Numerical inverse kinematics using Damped Least Squares (DLS).
    // Solves for joint angles that achieve the target end-effector pose.
    //   target_R, target_p : desired end-effector orientation and position
    //   q_init              : initial joint guess (size = nrOfJoints)
    //   q_out               : solved joint angles (size = nrOfJoints)
    //   max_iter            : maximum DLS iterations
    //   eps                 : convergence threshold on pose error norm
    //   lambda              : DLS damping factor (larger = more stable, less accurate)
    //   alpha               : step size for joint update
    //   use_nullspace       : if true, add null-space term to pull joints toward q_rest
    //   q_rest              : rest posture for null-space optimization (size = nrOfJoints)
    //   nullspace_gain      : gain for null-space term
    //   joint_weights       : diagonal W for weighted DLS (larger = move less); empty = identity
    //   position_weight     : task-space cost weight for translation error
    //   rotation_weight     : task-space cost weight for orientation error
    // Returns: final pose error norm (converged if < eps)
    double SolveIK(const Eigen::Matrix3d &target_R, const Eigen::Vector3d &target_p,
                   const std::vector<double> &q_init, std::vector<double> &q_out,
                   int max_iter = 50, double eps = 1e-4, double lambda = 0.05,
                   double alpha = 1.0, bool use_nullspace = false,
                   const std::vector<double> &q_rest = {}, double nullspace_gain = 0.01,
                   const std::vector<double> &joint_weights = {}, double position_weight = 0.5,
                   double rotation_weight = 0.5);
};
