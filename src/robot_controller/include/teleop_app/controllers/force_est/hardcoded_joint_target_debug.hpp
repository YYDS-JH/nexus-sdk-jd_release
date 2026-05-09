#ifndef TELEOP_APP__CONTROLLERS__HARDCODED_JOINT_TARGET_DEBUG_HPP_
#define TELEOP_APP__CONTROLLERS__HARDCODED_JOINT_TARGET_DEBUG_HPP_

#include <Eigen/Dense>

namespace teleop_app {
namespace controllers {

/**
 * @brief 调试：在冗余臂阻抗链路中，用各关节 bipolar 梯形波覆盖 q_target_extended / v_target_extended。
 *        平衡角为首次调用时 q_current_full 对应分量；q_i = q_eq_i + amp_i*s(t)，v_i = amp_i*ds/dt。
 *        关闭开关时立即返回，不修改向量。
 */
void maybeApplyHardcodedJointTargetDebug(
    int model_dof,
    const Eigen::VectorXd& q_current_full,
    Eigen::VectorXd& q_target_extended,
    Eigen::VectorXd& v_target_extended);

/** @brief 返回硬编码调试开关是否开启 */
bool isHardcodedJointTargetDebugEnabled();

/**
 * @brief 记录标定样本 (q, v, tau_ext)。硬编码调试开启时，首次调用自动启动标定定时器；
 *        到达标定时长后自动写入 CSV 并停止采集。不依赖业务状态机。
 */
void recordCalibrationSample(const Eigen::VectorXd& q,
                             const Eigen::VectorXd& v,
                             const Eigen::VectorXd& tau_ext);

/** @brief 设置标定采集时长（秒），须在首次采集前调用 */
void setCalibrationDuration(double sec);

/** @brief 标定是否正在进行中 */
bool isCalibrationActive();

}  // namespace controllers
}  // namespace teleop_app

#endif
