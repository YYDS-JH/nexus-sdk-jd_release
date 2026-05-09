#ifndef TELEOP_APP__CONTROLLERS__IK_QP_WLS_TASK_SLACK_SOLVER_HPP_
#define TELEOP_APP__CONTROLLERS__IK_QP_WLS_TASK_SLACK_SOLVER_HPP_
/**
 * @file ik_qp_wls_task_slack_solver.hpp
 * @brief 单层凸 QP 迭代 IK：决策变量 z = [Δq; s]，加权末端 + 任务松弛。
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
 * @brief 单层凸 QP 迭代 IK：决策变量 z = [Δq; s]，
 *        最小化加权 ‖JΔq + s - e‖²_W + ρ‖s‖² + w_ns‖Δq-Δq_ns‖² + w_reg‖Δq‖²。
 */
bool ikNullspaceSolveIteratorQpWlsTaskSlack(
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

inline bool ikNullspaceSolveIteratorQpWlsTaskSlack(
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
    return ikNullspaceSolveIteratorQpWlsTaskSlack(
        model, data, ee_frame_id, ik_config, init_pos, pinocchio::SE3(target_pose),
        q_out, q_ref, joint_lower_default, joint_upper_default,
        ik_joint_min, ik_joint_max, ik_joint_clamp_margin, diag);
}

}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__IK_QP_WLS_TASK_SLACK_SOLVER_HPP_
