#pragma once

#include "teleop_app/controllers/ik/ar5_analytical_ik_types.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace teleop_app {
namespace controllers {

/**
 * @file ar5_analytical_ik.hpp
 * @brief AR5（7R）末端 **link7** 的半解析逆运动学 + 臂型角 ψ 约定（与数值 IK 同一 EE frame）。
 *
 * ## 臂型角 ψ 与参考平面（对应冗余臂「自运动」与帖 2.2 节思想）
 *
 * - 末端位姿固定在 SE(3) 上时，7 轴仍可在雅可比零空间内运动（自运动），不改变末端位姿。
 * - 用 **臂型角 ψ** 参数化这一维冗余：ψ 表示「当前臂平面」与「参考臂平面」之间的夹角，
 *   二者均以 **肩–腕轴** e 为公共边（两平面交线）。
 *
 * ### 参考平面 Π_ref 的构造（帖中：θ3=0 + 虚拟非冗余臂）
 *
 * 1) **固定第三关节** θ3=0（与帖中「固定关节轴 3」一致）。
 * 2) 本机型 **关节 2 与 4 的轴线相互平行**（URDF 中均为局部 Y 轴，且结构使两轴保持平行，属常见 7R 设计）。
 * 3) 在此约束下，仅由 (θ1, θ2, θ4) 与腕部锁零（θ5=θ6=θ7=0）即可将 **link7 原点（腕心）**
 *    送至期望位置 p_w（位置子问题 3 元 3 程）。对应构型确定一条 **臂平面** Π_ref：
 *    由 e 与肘部相对肩部的法向（肘在肩–腕连线以外的一维偏置）张成。
 * 4) 将该平面法向在垂直于 e 的平面内归一化，得到 **n_ref**（参考平面的法向，用于数值稳定）。
 *
 * 实现中：VirtualBase::solveWristPositionQ3Zero 用阻尼牛顿法求 (q1,q2,q4)，得到 n_ref（与帖子「虚拟平面」一致）。
 *
 * ### 目标臂平面与 ψ 的施加
 *
 * - e = normalize(p_w - p_s)，p_s 为**肩球心**（与 URDF/脚本一致：三肩轴交点 —— 用 **link2 系原点**在世界系下的位置近似，q=0 时为 (0,0,d1)）。
 * - n_ref 由虚拟构型算得，且满足 n_ref ⟂ e。
 * - **目标法向** n_tgt = R_e(ψ) n_ref，其中 R_e(ψ) 为绕单位轴 e 的罗德里格斯旋转（保持 e 不动，因此在 ⊥e 平面内转动「臂平面」）。
 * - **物理含义**：ψ 为当前臂平面相对参考平面绕 e 的转角；自运动时末端不动，肘部绕 e 扫过，ψ 连续变化。
 *
 * ### 求解流程（半解析）
 *
 * 1) n_ref ← θ3=0 虚拟腕位到位；
 * 2) n_tgt ← R_e(ψ) n_ref；
 * 3) 对 (q1…q4) 做阻尼 GN：残差为腕位 p_w 与平面约束（n_cur≈n_tgt，n_cur 由当前 q 的肘/肩几何算得）；腕部 (q5,q6,q7) 由 R_0^4^T R_des 做 Z–Y–X（RzRyRx）闭式分解；
 * 4) 与 q_hint 距离最近的解；可选校验 FK(link7)。
 */

Ar5AnalyticalIkResult solveAr5AnalyticalIkLink7(
    const pinocchio::Model& model,
    const Eigen::Matrix4d& T_base_link7,
    double arm_angle_psi,
    const Eigen::VectorXd& q_hint,
    const Ar5AnalyticalIkSettings& settings = Ar5AnalyticalIkSettings{});

/**
 * @param model,data   与末端 frame 一致的 Pinocchio 模型（data 由调用方持有，避免内部分配）
 * @param phi_prev     上一时刻臂角；在 [phi_prev − half_width, phi_prev + half_width] 内按 step 枚举 Δφ
 * @param weight_ik    与 RobotModel::computeWeightedIkPoseErrorNorm 相同的 6 维权重
 *
 * **确定 φ 的策略**：对每个候选 φ，先半解析求 q，再用加权 pose 误差阈值筛解；在可行解中最小化
 * score = −μ + w_lim·H + w_s·(Δφ)²，其中 μ=√det(JJᵀ)（Yoshikawa），H 为关节限位裕量平方罚，Δφ=φ−φ_prev。
 */
Ar5PhiScanResult selectPhiLocalSearchAr5(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::Matrix4d& T_base_link7,
    double phi_prev,
    const Eigen::VectorXd& q_hint,
    const Eigen::Matrix<double, 6, 1>& weight_ik,
    const Ar5AnalyticalIkSettings& analytic_settings,
    const Ar5PhiScanSettings& scan_settings);

/**
 * 关节连续性 ψ 扫描 + 半解析 IK，输出数值 IK 用 q 初值。
 * @param q_joint_ref_full  用于 weight_joint_ref·‖q−q_ref‖²；size≠nq 或 weight_joint_ref==0 时不参与打分。
 *                        可与 q_hint_full 相同（例如都取上一周期数值 IK 解）；此时若 w_ref>0 与 w_joint_prev 同时较大，
 *                        与「上一拍关节」相关的平滑项会部分重叠，宜调小其一或只开一侧。
 * @param state_io          phi 与 q_tm1/q_tm2 由本函数按 settings 更新（首帧 phi 来自 seed.phi_init_rad 若未初始化）
 */
/// @param out_psi_used 若非空且返回 true，写入本拍选用的臂型角 ψ [rad]（便于日志；与是否写回 state_io.phi_prev 无关）
/// @param out_analytic_meta 若非空且返回 true，写入对应半解析解的 ok/diagnostic/q（与内部选中候选一致）
bool computeAr5PsiContinuitySeedForNumericIk(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::Matrix4d& T_des,
    const Eigen::VectorXd& q_hint_full,
    const Eigen::VectorXd& q_joint_ref_full,
    const Ar5AnalyticalIkSettings& analytic_settings,
    const Ar5PsiContinuitySeedSettings& continuity_settings,
    Ar5AnalyticalIkSeedRuntimeState& state_io,
    Eigen::VectorXd& q_init_out_full,
    double* out_psi_used = nullptr,
    Ar5AnalyticalIkResult* out_analytic_meta = nullptr);

/**
 * 按 cfg.seed_policy 选择 Yoshikawa φ 扫描或关节连续性 ψ 扫描，输出数值 IK 初值。
 * q_hint_full：建议为上一拍数值解或实测关节；q_joint_ref_full：贴参考姿态（可与 hint 相同或独立）。
 */
bool computeAr5AnalyticalIkSeedForNumericIk(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Ar5AnalyticalIkControlConfig& cfg,
    Ar5AnalyticalIkSeedRuntimeState& state_io,
    const Eigen::VectorXd& q_hint_full,
    const Eigen::VectorXd& q_joint_ref_full,
    const Eigen::Matrix4d& T_des,
    const Eigen::Matrix<double, 6, 1>& weight_ik,
    Eigen::VectorXd& q_init_out_full);

}  // namespace controllers
}  // namespace teleop_app
