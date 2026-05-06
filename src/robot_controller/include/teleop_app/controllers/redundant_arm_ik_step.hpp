#pragma once

#include <Eigen/Dense>

#include "teleop_app/controllers/data_types.hpp"
#include "teleop_app/controllers/robot_model.hpp"

namespace teleop_app {
namespace controllers {

/**
 * @brief 冗余臂单周期 IK 步输出（关节盒 + hold-skip + 数值 IK；不含 FK 调试写缓存）。
 */
struct RedundantArmIkStepResult {
    Eigen::VectorXd q_des;
    Eigen::VectorXd q_ik_full;
    bool ik_ok{false};
};

/// 跨周期持久：上一周期 IK 步的关节输出（含未收敛时的末步迭代值、hold-skip 时为 q_current）；由 ArmController 持有。
struct RedundantArmIkRuntimeState {
    Eigen::VectorXd last_ik_solution;
    bool has_last_ik_solution{false};
    /// redundant_numeric_ik_init=analytic_theory 时：ψ/φ 与 q 历史（与 ar5_analytical_ik 库共用）
    Ar5AnalyticalIkSeedRuntimeState ar5_seed_state{};
};

/**
 * redundant_ik_safety 关节盒、hold-skip、solveInverseKinematicsIterator。
 *
 * @param params IK / redundant_ik_safety
 * @param runtime 可改写跨周期状态（last_ik 等）
 */
RedundantArmIkStepResult runRedundantArmInverseKinematicsStep(
    RobotModel& robot_model,
    const ControllerParams& params,
    RedundantArmIkRuntimeState& runtime,
    const Eigen::VectorXd& q_current_full,
    const Eigen::Matrix4d& desired_pose_for_ik,
    const Eigen::VectorXd& q_target_locked_dof,
    int locked_dof);

}  // namespace controllers
}  // namespace teleop_app
