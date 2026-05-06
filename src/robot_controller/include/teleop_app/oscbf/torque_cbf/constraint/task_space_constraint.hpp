#ifndef TASK_SPACE_TORQUE_CONSTRAINT_HPP
#define TASK_SPACE_TORQUE_CONSTRAINT_HPP

#include "teleop_app/oscbf/torque_cbf/constraint/base_constraint.hpp"
#include <Eigen/Dense>
#include <vector>

namespace OSCBF {
namespace torque_cbf {

/**
 * @brief Task space constraint using torque-based CBF
 * 
 * 基于力矩的任务空间约束，使用二阶CBF：
 * - h = pos - pos_min (for lower bound) or pos_max - pos (for upper bound)
 * - h2 = Lf_h + alpha * h
 * - Lg_h2 = grad_h_pos^T @ J_ee @ M^{-1}
 * - 约束：Lg_h2 @ τ >= -alpha2 * h2 - Lf_h2
 */
class TaskSpaceTorqueConstraint : public BaseTorqueCBFConstraint {
public:
    /**
     * @brief Task space limit information
     */
    struct TaskSpaceLimitInfo {
        double h;
        double h2;
        Eigen::VectorXd Lg_h2;
        double cbf_rhs;
        std::string axis;      // "x", "y", or "z"
        std::string limit_type; // "min" or "max"
    };

    /**
     * @brief Initialize task space torque constraint
     * 
     * @param name Constraint name
     * @param pos_min Minimum position limits (3D: [x_min, y_min, z_min])
     * @param pos_max Maximum position limits (3D: [x_max, y_max, z_max])
     * @param safety_margin Safety margin from limits (m)
     * @param alpha First-order CBF coefficient
     * @param alpha2 Second-order CBF coefficient (torque control requires alpha2)
     */
    TaskSpaceTorqueConstraint(
        const std::string& name = "task_space",
        const Eigen::Vector3d& pos_min = Eigen::Vector3d(0.3, -0.1, 0.0),
        const Eigen::Vector3d& pos_max = Eigen::Vector3d(0.5, 0.1, 0.4),
        double safety_margin = 0.01,
        double alpha = 50.0,
        double alpha2 = 50.0
    );

    /**
     * @brief Compute CBF (implements base class pure virtual function)
     * 
     * Note: This method requires additional information (pos_ee, vel_ee, J_ee)
     * which must be computed externally. This is a stub implementation that
     * throws an error. Use compute_cbf(pos_ee, vel_ee, J_ee, q, v, a, M) instead.
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @return CBFResult (throws if called directly)
     */
    CBFResult compute_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    ) override;

    /**
     * @brief Compute task space CBF for the most critical constraint
     * 
     * @param pos_ee End-effector position (3D)
     * @param vel_ee End-effector velocity (3D)
     * @param J_ee End-effector Jacobian (3x6)
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @return CBFResult for the most critical constraint
     */
    CBFResult compute_cbf(
        const Eigen::Vector3d& pos_ee,
        const Eigen::Vector3d& vel_ee,
        const Eigen::MatrixXd& J_ee,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    );

    /**
     * @brief Compute CBF for all task space limits
     * 
     * @param pos_ee End-effector position (3D)
     * @param vel_ee End-effector velocity (3D)
     * @param J_ee End-effector Jacobian (3x6)
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @return Vector of TaskSpaceLimitInfo for all limits
     */
    std::vector<TaskSpaceLimitInfo> compute_all_task_space_cbfs(
        const Eigen::Vector3d& pos_ee,
        const Eigen::Vector3d& vel_ee,
        const Eigen::MatrixXd& J_ee,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    );

    // Getters
    Eigen::Vector3d getPosMin() const { return pos_min_; }
    Eigen::Vector3d getPosMax() const { return pos_max_; }
    double getSafetyMargin() const { return safety_margin_; }

private:
    Eigen::Vector3d pos_min_;
    Eigen::Vector3d pos_max_;
    double safety_margin_;
};

} // namespace torque_cbf
} // namespace OSCBF

#endif // TASK_SPACE_TORQUE_CONSTRAINT_HPP

