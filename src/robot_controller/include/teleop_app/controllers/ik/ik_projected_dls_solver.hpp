#ifndef TELEOP_APP__CONTROLLERS__IK_PROJECTED_DLS_SOLVER_HPP_
#define TELEOP_APP__CONTROLLERS__IK_PROJECTED_DLS_SOLVER_HPP_
/**
 * @file ik_projected_dls_solver.hpp
 * @brief 投影阻尼最小二乘迭代 IK + 显式零空间投影。
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
 * @brief 投影阻尼最小二乘迭代 IK + 零空间投影（SE3 + log6，LOCAL 雅可比 + Jlog6）。
 *
 * @param ee_frame_id 末端 frame 索引；<0 时使用最后一个关节
 * @param q_out       成功或失败时均为最后一次迭代的 nv 维关节向量
 * @param q_ref       零空间关节参考项
 */
bool ikNullspaceSolveIteratorProjectedDls(
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

inline bool ikNullspaceSolveIteratorProjectedDls(
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
    return ikNullspaceSolveIteratorProjectedDls(
        model, data, ee_frame_id, ik_config, init_pos, pinocchio::SE3(target_pose),
        q_out, q_ref, joint_lower_default, joint_upper_default,
        ik_joint_min, ik_joint_max, ik_joint_clamp_margin, diag);
}

}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__IK_PROJECTED_DLS_SOLVER_HPP_
