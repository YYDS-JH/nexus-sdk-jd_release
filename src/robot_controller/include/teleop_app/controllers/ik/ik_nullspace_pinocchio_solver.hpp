#ifndef TELEOP_APP__CONTROLLERS__IK_NULLSPACE_PINOCCHIO_SOLVER_HPP_
#define TELEOP_APP__CONTROLLERS__IK_NULLSPACE_PINOCCHIO_SOLVER_HPP_
/**
 * @file ik_nullspace_pinocchio_solver.hpp
 * @brief 聚合头文件：包含所有数值 IK 求解器与公共类型。
 *
 * 各求解器独立头文件：
 *   - ik_solver_common.hpp                 — 公共诊断类型、关节速度工具
 *   - ik_projected_dls_solver.hpp          — 阻尼广义逆 + 显式零空间投影
 *   - ik_qp_wls_task_slack_solver.hpp      — 凸 QP [Δq; s] + 任务松弛
 *   - ik_velocity_qp_solver.hpp            — 速度级 QP + CLIK [q̇; s]
 */

#include "teleop_app/controllers/ik/ik_solver_common.hpp"
#include "teleop_app/controllers/ik/ik_projected_dls_solver.hpp"
#include "teleop_app/controllers/ik/ik_qp_wls_task_slack_solver.hpp"
#include "teleop_app/controllers/ik/ik_velocity_qp_solver.hpp"

#endif  // TELEOP_APP__CONTROLLERS__IK_NULLSPACE_PINOCCHIO_SOLVER_HPP_
