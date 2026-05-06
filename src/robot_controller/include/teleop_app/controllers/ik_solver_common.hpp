#ifndef TELEOP_APP__CONTROLLERS__IK_SOLVER_COMMON_HPP_
#define TELEOP_APP__CONTROLLERS__IK_SOLVER_COMMON_HPP_
/**
 * @file ik_solver_common.hpp
 * @brief IK 求解器公共类型与工具函数（各数值算法共享）。
 */

#include <vector>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "teleop_app/controllers/data_types.hpp"

namespace teleop_app {
namespace controllers {

/**
 * @brief 与 RobotModel 原 IK 迭代器一致的数值诊断（可选）。
 */
struct IkNullspacePinocchioDiagnostics {
    int iterations = 0;
    bool converged = false;
    double final_weighted_error_norm = 0.0;
};

/**
 * @brief 与 ArmController 冗余臂链路一致：v_nom(i) = clamp((q_des(i)-q_track(i))/ctrl_dt, ±max_velocity)。
 *        q_track 常为虚拟参考 q_target；维数须与 q_des 一致。
 */
void computeNominalJointVelocityForCbf(
    const Eigen::VectorXd& q_des,
    const Eigen::VectorXd& q_track,
    const ControllerParams& params,
    Eigen::VectorXd& v_nom);

/**
 * @brief IK（或其它来源）给出的关节目标位置 + 按控制周期差分得到的名义速度。
 */
struct JointPositionVelocityReference {
    Eigen::VectorXd q;  ///< 目标关节位置 (rad)
    Eigen::VectorXd v;  ///< 目标关节速度 (rad/s)
};

void fillJointPositionVelocityReference(
    const Eigen::VectorXd& q_des,
    const Eigen::VectorXd& q_track,
    const ControllerParams& params,
    JointPositionVelocityReference& out);

}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__IK_SOLVER_COMMON_HPP_
