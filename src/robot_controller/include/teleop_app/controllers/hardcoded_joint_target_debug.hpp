#ifndef TELEOP_APP__CONTROLLERS__HARDCODED_JOINT_TARGET_DEBUG_HPP_
#define TELEOP_APP__CONTROLLERS__HARDCODED_JOINT_TARGET_DEBUG_HPP_

#include <Eigen/Dense>

namespace teleop_app {
namespace controllers {

/**
 * @brief 调试：在冗余臂阻抗链路中，用各关节 bipolar 梯形波覆盖 q_target_extended / v_target_extended。
 *        平衡角为首次调用时 q_current_full 对应分量；q_i = q_eq_i + amp_i*s(t)，v_i = amp_i*ds/dt。
 *        关闭开关时立即返回，不修改向量。
 * @param model_dof URDF 可动维数（与 q_current_full 一致）
 * @param q_current_full 当前关节位置（全维）；首帧锁平衡；尾部关节照抄当前位、速度 0
 * @param q_target_extended in/out
 * @param v_target_extended in/out
 */
void maybeApplyHardcodedJointTargetDebug(
    int model_dof,
    const Eigen::VectorXd& q_current_full,
    Eigen::VectorXd& q_target_extended,
    Eigen::VectorXd& v_target_extended);

}  // namespace controllers
}  // namespace teleop_app

#endif
