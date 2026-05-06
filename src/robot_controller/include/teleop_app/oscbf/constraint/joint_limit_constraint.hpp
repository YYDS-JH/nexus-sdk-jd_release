#ifndef JOINT_LIMIT_CONSTRAINT_HPP
#define JOINT_LIMIT_CONSTRAINT_HPP

#include "base_constraint.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace OSCBF {

/**
 * @brief Joint limit constraint for velocity-based control
 * 
 * 关节限位CBF（Joint limit avoidance）：
 * 确保关节位置在指定的限位范围内。
 */
class JointLimitVelocityConstraint : public BaseVelocityCBFConstraint {
public:
    /**
     * @brief Initialize joint limit constraint
     * 
     * @param name Constraint name
     * @param joint_limits_min Minimum joint limits (rad), shape: (n_joints,)
     * @param joint_limits_max Maximum joint limits (rad), shape: (n_joints,)
     * @param safety_margin Safety margin added to limits (rad)
     * @param alpha CBF coefficient
     * @param n_joints Number of joints
     */
    JointLimitVelocityConstraint(
        const std::string& name = "joint_limit",
        const Eigen::VectorXd& joint_limits_min = Eigen::VectorXd(),
        const Eigen::VectorXd& joint_limits_max = Eigen::VectorXd(),
        double safety_margin = 0.01,
        double alpha = 50.0,
        int n_joints = 6,
        double interior_cbf_deactivate_margin = 0.0
    );

    /**
     * @brief Compute joint limit CBF for velocity control
     * 
     * 计算所有关节的上下界约束，返回最紧急的约束。
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @return CBFResult for the most critical constraint
     */
    CBFResult compute_cbf(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v
    );

    /**
     * @brief Compute all joint limit CBF constraints (for all joints and bounds)
     * 
     * @param q Joint positions (n_joints,)
     * @param v Joint velocities (n_joints,)
     * @return Vector of constraint info with joint index and bound type
     */
    struct ConstraintInfo {
        CBFResult result;
        int joint_idx;
        std::string bound_type;  // "min" or "max"
    };
    std::vector<ConstraintInfo> compute_all_joint_limit_cbfs(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v
    );

    // Override pure virtual function (not used, but required)
    CBFResult compute_cbf() override {
        throw std::runtime_error("JointLimitVelocityConstraint::compute_cbf() requires parameters");
    }

    // Getters
    double getSafetyMargin() const { return safety_margin_; }
    int getNJoints() const { return n_joints_; }
    Eigen::VectorXd getJointLimitsMin() const { return joint_limits_min_; }
    Eigen::VectorXd getJointLimitsMax() const { return joint_limits_max_; }

private:
    double safety_margin_;
    int n_joints_;
    Eigen::VectorXd joint_limits_min_;
    Eigen::VectorXd joint_limits_max_;
};

} // namespace OSCBF

#endif // JOINT_LIMIT_CONSTRAINT_HPP

