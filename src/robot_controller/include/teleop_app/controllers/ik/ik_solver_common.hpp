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
 * @brief IK 内部平面障碍物关节锁定诊断。
 */
struct IkObstaclePlaneDiagnostics {
    bool enabled = false;
    bool active = false;
    int num_locked = 0;
    int num_free = 0;
    double min_signed_distance = 0.0;
    double freeze_threshold = 0.0;
    std::vector<int> locked_joint_indices;
    /// 与 checked_joint_indices 一一对应的有符号平面距离
    std::vector<int> checked_joint_indices;
    std::vector<double> link_signed_distances;
};

/**
 * @brief 与 RobotModel 原 IK 迭代器一致的数值诊断（可选）。
 */
struct IkNullspacePinocchioDiagnostics {
    int iterations = 0;
    bool converged = false;
    double final_weighted_error_norm = 0.0;
    IkObstaclePlaneDiagnostics obstacle;
};

/**
 * @brief 编码 IK 障碍物 debug 向量，供 /debug/ik_obstacle 发布。
 * Layout: [active, num_locked, num_free, min_dist, freeze_thresh,
 *          converged, iterations, final_err,
 *          locked_mask_0..locked_mask_{nv-1},
 *          link_dist for checked joints (variable length, padded to max_obs_slots)]
 */
Eigen::VectorXd encodeIkObstacleDebugVector(
    const IkNullspacePinocchioDiagnostics& diag,
    int nv,
    int max_obs_slots = 8);

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
