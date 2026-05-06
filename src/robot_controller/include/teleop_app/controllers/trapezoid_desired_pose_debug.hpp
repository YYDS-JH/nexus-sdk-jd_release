#ifndef TELEOP_APP__CONTROLLERS__TRAPEZOID_DESIRED_POSE_DEBUG_HPP_
#define TELEOP_APP__CONTROLLERS__TRAPEZOID_DESIRED_POSE_DEBUG_HPP_

#include <Eigen/Dense>

namespace teleop_app {
namespace controllers {

/**
 * @brief 调试：首次 FK 存为世界系平衡 (t_eq, R_eq)。每通道为双平台对称梯形（± 幅值，周期内两段零斜率保持），
 *        t_des=t_eq+Δx_w，R_des=exp3(ω_w)*R_eq。合成 T_des 替代外部期望。
 * @param desired_pose_external 关闭硬编码时原样返回。
 * @param T_ee_current_fk 当前关节下末端 frame（RobotModel 配置的 ee，如 AR5 为 link7）的 FK；首次调用时锁定平衡位姿。
 */
Eigen::Matrix4d makeDebugTrapezoidDesiredPose(
    const Eigen::Matrix4d& desired_pose_external,
    const Eigen::Matrix4d& T_ee_current_fk);

}  // namespace controllers
}  // namespace teleop_app

#endif
