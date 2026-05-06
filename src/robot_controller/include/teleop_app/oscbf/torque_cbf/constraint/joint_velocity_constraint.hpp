#ifndef JOINT_VELOCITY_TORQUE_CONSTRAINT_HPP
#define JOINT_VELOCITY_TORQUE_CONSTRAINT_HPP

#include "teleop_app/oscbf/torque_cbf/constraint/base_constraint.hpp"
#include <Eigen/Dense>
#include <vector>

namespace OSCBF {
namespace torque_cbf {

/**
 * @brief Joint velocity constraint using torque-based CBF
 * 
 * 基于力矩的关节速度约束，使用一阶CBF（相对度为1）：
 * - h = v - v_min - safety_margin (for lower bound) or v_max - v - safety_margin (for upper bound)
 * - 约束：Lg_h @ τ >= -α * h - Lf_h
 * 
 * 物理意义：
 * - 限制关节速度在安全范围内
 * - 防止因过大速度导致的机械损伤
 * - 约束直接作用于力矩输入
 */
class JointVelocityTorqueConstraint : public BaseTorqueCBFConstraint {
public:
    /**
     * @brief Joint velocity limit information
     */
    struct JointVelocityInfo {
        double h;                  // CBF value
        double h2;                 // Second-order CBF value (for compatibility)
        Eigen::VectorXd Lg_h2;     // Lie derivative w.r.t. torque
        double cbf_rhs;            // Constraint RHS
        int joint_idx;             // Joint index
        std::string limit_type;    // "min" or "max"
    };

    /**
     * @brief Initialize joint velocity torque constraint
     * 
     * @param name Constraint name
     * @param velocity_limits_min Minimum joint velocities (n_joints,) [rad/s]
     * @param velocity_limits_max Maximum joint velocities (n_joints,) [rad/s]
     * @param safety_margin Safety margin from velocity limits (rad/s)
     * @param alpha CBF coefficient (larger = more aggressive constraint)
     * @param n_joints Number of joints
     */
    JointVelocityTorqueConstraint(
        const std::string& name = "joint_velocity",
        const Eigen::VectorXd& velocity_limits_min = Eigen::VectorXd(),
        const Eigen::VectorXd& velocity_limits_max = Eigen::VectorXd(),
        double safety_margin = 0.1,
        double alpha = 10.0,
        int n_joints = 6
    );

    /**
     * @brief Compute joint velocity CBF (implements base class pure virtual function)
     * 
     * @param q Joint positions (n_joints,) - not used, for interface compatibility
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
     * @brief Compute joint velocity CBF with Coriolis and gravity
     * 
     * @param v Joint velocities (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @param c Coriolis and centrifugal force vector (n_joints,)
     * @param g Gravity vector (n_joints,)
     * @return CBFResult for the most critical joint
     */
    CBFResult compute_cbf(
        const Eigen::VectorXd& v,
        const Eigen::MatrixXd& M,
        const Eigen::VectorXd& c,
        const Eigen::VectorXd& g
    );

    /**
     * @brief Compute CBF for all joint velocity limits
     * 
     * @param v Joint velocities (n_joints,)
     * @param M Joint space inertia matrix (n_joints x n_joints)
     * @param c Coriolis and centrifugal force vector (n_joints,)
     * @param g Gravity vector (n_joints,)
     * @return Vector of JointVelocityInfo for all joint velocity limits
     */
    std::vector<JointVelocityInfo> compute_all_joint_velocity_cbfs(
        const Eigen::VectorXd& v,
        const Eigen::MatrixXd& M,
        const Eigen::VectorXd& c = Eigen::VectorXd(),
        const Eigen::VectorXd& g = Eigen::VectorXd()
    );

    // Getters
    Eigen::VectorXd getVelocityLimitsMin() const { return velocity_limits_min_; }
    Eigen::VectorXd getVelocityLimitsMax() const { return velocity_limits_max_; }
    double getSafetyMargin() const { return safety_margin_; }
    int getNJoints() const { return n_joints_; }

    // Setters for runtime tuning
    void setVelocityLimits(const Eigen::VectorXd& min, const Eigen::VectorXd& max);
    void setSafetyMargin(double margin) { safety_margin_ = margin; }

private:
    Eigen::VectorXd velocity_limits_min_;
    Eigen::VectorXd velocity_limits_max_;
    double safety_margin_;
    int n_joints_;
};

} // namespace torque_cbf
} // namespace OSCBF

#endif // JOINT_VELOCITY_TORQUE_CONSTRAINT_HPP

