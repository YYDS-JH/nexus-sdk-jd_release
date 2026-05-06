#ifndef OSCBF_HPP
#define OSCBF_HPP

/**
 * @file oscbf.hpp
 * @brief Main header file for OSCBF (Operational Space Control Barrier Functions) library
 * 
 * This header includes all necessary components for using OSCBF in C++ projects.
 */

// Base constraint class
#include "constraint/base_constraint.hpp"

// Constraint implementations
#include "constraint/obstacle_avoidance_constraint.hpp"
#include "constraint/task_space_constraint.hpp"
#include "constraint/joint_limit_constraint.hpp"
#include "constraint/self_collision_constraint.hpp"

// Utils for collision pair manager
#include "utils/collision_pair_manager.hpp"
// QP solver
#include "solver/velocity_qp_solver.hpp"

// Main controller
#include "oscbf_controller.hpp"

/**
 * @namespace OSCBF
 * @brief Namespace for all OSCBF classes and functions
 */

#endif // OSCBF_HPP

