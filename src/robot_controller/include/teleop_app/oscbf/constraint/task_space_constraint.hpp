#ifndef TASK_SPACE_CONSTRAINT_HPP
#define TASK_SPACE_CONSTRAINT_HPP

#include "base_constraint.hpp"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

namespace OSCBF {

/**
 * @brief Task space containment constraint for velocity-based control
 * 
 * 任务空间限制CBF（End-effector containment）：
 * 确保末端执行器位置在指定的工作空间范围内。
 */
class TaskSpaceVelocityConstraint : public BaseVelocityCBFConstraint {
public:
    /**
     * @brief Structure to hold task space limits for each axis
     */
    struct AxisLimits {
        double min;
        double max;
    };

    /**
     * @brief Initialize task space containment constraint
     * 
     * @param name Constraint name
     * @param task_space_limits Map defining workspace limits for x, y, z axes
     * @param alpha CBF coefficient
     * @param safety_margin Safety margin added to limits (m)
     */
    TaskSpaceVelocityConstraint(
        const std::string& name = "task_space",
        const std::map<std::string, AxisLimits>& task_space_limits = {},
        double alpha = 50.0,
        double safety_margin = 0.01,
        double interior_cbf_deactivate_margin = 0.0
    );

    /**
     * @brief Compute task space containment CBF for velocity control
     * 
     * 速度控制下，相对度为1的一阶CBF（根据论文公式 ż = u）：
     * - 系统动力学：ż = u，其中 z = q（关节位置），u = q̇（关节速度命令）
     * - f(z) = 0（没有漂移项），g(z) = I（单位矩阵）
     * 
     * CBF定义：
     * - h = pos_ee - pos_min 或 h = pos_max - pos_ee
     * - ḣ = ∂h/∂pos_ee · J_ee · v
     * 
     * Lie导数：
     * - Lf_h = ∂h/∂q · f(z) = 0（因为 f(z) = 0）
     * - Lg_h = ∂h/∂q · g(z) = ∂h/∂pos_ee · J_ee（控制输入梯度）
     * 
     * 约束：Lg_h @ v >= -Lf_h - αh
     * 
     * 计算所有轴（x, y, z）的上下界约束，返回最紧急的约束。
     * 
     * @param pos_ee End-effector position (3D)
     * @param vel_ee End-effector velocity (3D) - 仅用于调试，不用于计算Lf_h
     * @param J_ee End-effector position Jacobian (3x6)
     * @return CBFResult for the most critical constraint
     */
    CBFResult compute_cbf(
        const Eigen::Vector3d& pos_ee,
        const Eigen::Vector3d& vel_ee,
        const Eigen::MatrixXd& J_ee
    );

    /**
     * @brief Compute all task space CBF constraints (for all axes and bounds)
     * 
     * @param pos_ee End-effector position (3D)
     * @param vel_ee End-effector velocity (3D)
     * @param J_ee End-effector position Jacobian (3x6)
     * @return Vector of CBFResult tuples with axis name and bound type
     */
    struct ConstraintInfo {
        CBFResult result;
        std::string axis_name;
        std::string bound_type;  // "min" or "max"
    };
    std::vector<ConstraintInfo> compute_all_task_space_cbfs(
        const Eigen::Vector3d& pos_ee,
        const Eigen::Vector3d& vel_ee,
        const Eigen::MatrixXd& J_ee
    );

    // Override pure virtual function (not used, but required)
    CBFResult compute_cbf() override {
        throw std::runtime_error("TaskSpaceVelocityConstraint::compute_cbf() requires parameters");
    }

    // Getters
    double getSafetyMargin() const { return safety_margin_; }
    Eigen::Vector3d getPosMin() const { return pos_min_; }
    Eigen::Vector3d getPosMax() const { return pos_max_; }

private:
    double safety_margin_;
    Eigen::Vector3d pos_min_;
    Eigen::Vector3d pos_max_;
};

} // namespace OSCBF

#endif // TASK_SPACE_CONSTRAINT_HPP

