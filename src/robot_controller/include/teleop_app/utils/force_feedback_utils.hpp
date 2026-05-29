#pragma once

#include <cmath>

#include "teleop_app/controllers/data_types.hpp"

namespace teleop_app {
namespace utils {

/// 1 关节复位角有效判定阈值 (rad)：|j1| 低于此值视为“无明确安装方向”，rotation 置 0
constexpr double kEeFfRotationWristJ1MinAbsRad = 0.5;

/// 力反馈 xy 平面标准补偿角 (deg)，与 AR5 基座 ±90° 安装对应
constexpr double kEeFfRotationZStandardDeg = 90.0;

/**
 * @brief 根据从臂 wrist 1 关节复位角推导力反馈绕 Z 轴旋转角 (deg)
 *
 * 规则（与 nexus_manage end_effector_configs.*.robot_pos_values[0] 一致）：
 *   j1 ≈ -π/2 (-1.57) → +90°
 *   j1 ≈ +π/2 (+1.57) → -90°
 *   |j1| 过小          → 0°（不旋转）
 */
inline double computeEeFfRotationZDegFromWristJ1(double wrist_j1_rad) {
    if (std::abs(wrist_j1_rad) < kEeFfRotationWristJ1MinAbsRad) {
        return 0.0;
    }
    return -std::copysign(kEeFfRotationZStandardDeg, wrist_j1_rad);
}

/**
 * @brief 对 6D 笛卡尔力/力矩的 xy 分量施加绕 Z 轴旋转（与 arm_controller 主臂接收侧公式一致）
 */
inline void applyEeFfRotationZInPlace(controllers::CartesianForce& force, double rotation_z_deg) {
    if (std::abs(rotation_z_deg) <= 1e-9) {
        return;
    }
    const double theta = rotation_z_deg * M_PI / 180.0;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    const double fx = force.fx;
    const double fy = force.fy;
    force.fx = c * fx - s * fy;
    force.fy = s * fx + c * fy;

    const double mx = force.mx;
    const double my = force.my;
    force.mx = c * mx - s * my;
    force.my = s * mx + c * my;
}

}  // namespace utils
}  // namespace teleop_app
