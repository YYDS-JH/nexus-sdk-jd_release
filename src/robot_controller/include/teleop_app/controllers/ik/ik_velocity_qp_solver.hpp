#ifndef TELEOP_APP__CONTROLLERS__IK_VELOCITY_QP_SOLVER_HPP_
#define TELEOP_APP__CONTROLLERS__IK_VELOCITY_QP_SOLVER_HPP_
/**
 * @file ik_velocity_qp_solver.hpp
 * @brief 速度级 QP + CLIK 迭代 IK：决策变量 z = [q̇; s]，
 *        关节速度盒约束 + 位置约束速度化 + 可选末端速度限制。
 */

#include <vector>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "teleop_app/controllers/data_types.hpp"
#include "teleop_app/controllers/ik/ik_solver_common.hpp"

namespace teleop_app {
namespace controllers {

/**
 * @brief 速度级 QP 迭代 IK（CLIK）。
 *
 * 目标函数：
 *   min  ‖W(Jq̇ + s − ẋ_des)‖² + ρ‖s‖²
 *        + λ‖q̇‖²  + w_ref‖q̇ − q̇_ref‖²
 *        − α∇w(q)ᵀq̇  + β∇(1/w(q))ᵀq̇
 *
 * 约束：
 *   - 关节速度盒：max(−q̇_max, k(q_min−q)) ≤ q̇ ≤ min(q̇_max, k(q_max−q))
 *   - 末端速度限制（可选）：−ẋ_max ≤ Jq̇ ≤ ẋ_max
 *   - 可操作度 CBF（可选）：dt·∇μᵀq̇ ≥ γ(μ_min − μ)
 */
bool ikNullspaceSolveIteratorVelocityQp(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    int ee_frame_id,
    const IKConfig& ik_config,
    const Eigen::VectorXd& init_pos,
    const pinocchio::SE3& T_des,
    Eigen::VectorXd& q_out,
    const Eigen::VectorXd& q_ref,
    const std::vector<double>& joint_lower_default,
    const std::vector<double>& joint_upper_default,
    const Eigen::VectorXd* ik_joint_min = nullptr,
    const Eigen::VectorXd* ik_joint_max = nullptr,
    double ik_joint_clamp_margin = -1.0,
    IkNullspacePinocchioDiagnostics* diag = nullptr);

inline bool ikNullspaceSolveIteratorVelocityQp(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    int ee_frame_id,
    const IKConfig& ik_config,
    const Eigen::VectorXd& init_pos,
    const Eigen::Matrix4d& target_pose,
    Eigen::VectorXd& q_out,
    const Eigen::VectorXd& q_ref,
    const std::vector<double>& joint_lower_default,
    const std::vector<double>& joint_upper_default,
    const Eigen::VectorXd* ik_joint_min = nullptr,
    const Eigen::VectorXd* ik_joint_max = nullptr,
    double ik_joint_clamp_margin = -1.0,
    IkNullspacePinocchioDiagnostics* diag = nullptr) {
    return ikNullspaceSolveIteratorVelocityQp(
        model, data, ee_frame_id, ik_config, init_pos, pinocchio::SE3(target_pose),
        q_out, q_ref, joint_lower_default, joint_upper_default,
        ik_joint_min, ik_joint_max, ik_joint_clamp_margin, diag);
}

}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__IK_VELOCITY_QP_SOLVER_HPP_
