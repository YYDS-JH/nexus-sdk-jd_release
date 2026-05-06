#ifndef JOINT_LIMIT_TORQUE_CONSTRAINT_HPP
#define JOINT_LIMIT_TORQUE_CONSTRAINT_HPP

#include "teleop_app/oscbf/torque_cbf/constraint/base_constraint.hpp"
#include <Eigen/Dense>
#include <vector>

namespace OSCBF {
namespace torque_cbf {

/**
 * @brief Joint limit constraint using torque-based CBF
 * 
 * 基于力矩的关节限位约束，使用二阶CBF：
 * - h = q - q_min - safety_margin (for lower bound) or q_max - q - safety_margin (for upper bound)
 * - h2 = Lf_h + alpha * h
 * - Lg_h2 = direction * e_j^T @ M^{-1}
 * - 约束：Lg_h2 @ τ >= -alpha2 * h2 - Lf_h2
 */
class JointLimitTorqueConstraint : public BaseTorqueCBFConstraint {
public:
    /**
     * @brief Joint limit information
     */
    struct JointLimitInfo {
        double h;
        double h2;
        Eigen::VectorXd Lg_h2;
        double cbf_rhs;
        int joint_idx;
        std::string limit_type; // "min" or "max"
    };

    /**
     * @brief Initialize joint limit torque constraint
     * 
     * @param name Constraint name
     * @param joint_limits_min Minimum joint angles (n_joints,)
     * @param joint_limits_max Maximum joint angles (n_joints,)
     * @param safety_margin Safety margin from joint limits (rad)
     * @param alpha First-order CBF coefficient
     * @param alpha2 Second-order CBF coefficient (torque control requires alpha2)
     * @param n_joints Number of joints
     */
    JointLimitTorqueConstraint(
        const std::string& name = "joint_limit",
        const Eigen::VectorXd& joint_limits_min = Eigen::VectorXd(),
        const Eigen::VectorXd& joint_limits_max = Eigen::VectorXd(),
        double safety_margin = 0.01,
        double alpha = 50.0,
        double alpha2 = 50.0,
        int n_joints = 6
    );

    /**
     * @brief Compute joint limit CBF (implements base class pure virtual function)
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @return CBFResult for the most critical joint
     */
    CBFResult compute_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M
    ) override;

    /**
     * @brief Compute joint limit CBF for the most critical joint (with optional c and g)
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @param c Coriolis and centrifugal force vector (n_joints,) - optional, for computing intrinsic acceleration
     * @param g Gravity vector (n_joints,) - optional, for computing intrinsic acceleration
     * @return CBFResult for the most critical joint
     */
    CBFResult compute_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M,
        const Eigen::VectorXd& c,
        const Eigen::VectorXd& g
    );

    /**
     * @brief Compute CBF for all joint limits
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @param a Joint accelerations (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @param c Coriolis and centrifugal force vector (n_joints,) - optional
     * @param g Gravity vector (n_joints,) - optional
     * @return Vector of JointLimitInfo for all joint limits
     */
    std::vector<JointLimitInfo> compute_all_joint_limit_cbfs(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        const Eigen::MatrixXd& M,
        const Eigen::VectorXd& c = Eigen::VectorXd(),
        const Eigen::VectorXd& g = Eigen::VectorXd()
    );

    // Getters
    Eigen::VectorXd getJointLimitsMin() const { return joint_limits_min_; }
    Eigen::VectorXd getJointLimitsMax() const { return joint_limits_max_; }
    double getSafetyMargin() const { return safety_margin_; }
    int getNJoints() const { return n_joints_; }

private:
    Eigen::VectorXd joint_limits_min_;
    Eigen::VectorXd joint_limits_max_;
    double safety_margin_;
    int n_joints_;
};

} // namespace torque_cbf
} // namespace OSCBF

#endif // JOINT_LIMIT_TORQUE_CONSTRAINT_HPP

