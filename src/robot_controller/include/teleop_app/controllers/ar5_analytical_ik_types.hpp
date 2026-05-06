#pragma once

#include <string>

#include <Eigen/Core>

namespace teleop_app {
namespace controllers {

/**
 * 配置与结果类型（无 Pinocchio 依赖）：供 YAML / ControllerParams 与解析库共用。
 * 臂型角 ψ/φ 的几何含义见 ar5_analytical_ik.hpp 注释。
 */
struct Ar5AnalyticalIkSettings {
    std::string ee_link_name{"AR5-5_07L-W4C4A2_link7"};
    std::string shoulder_ref_link_name{"AR5-5_07L-W4C4A2_link2"};
    std::string elbow_ref_link_name{"AR5-5_07L-W4C4A2_link4"};
    int max_outer_iters{40};
    double lm_lambda_init{1e-2};
    double pos_tol{1e-5};
    double plane_tol{1e-3};
    double plane_residual_weight{5.0};
    /// FK 校验：||p_ee−p_des|| < 该值且姿态误差 < ok_rot_tol_rad 时允许 ok（且须 vref 成功）
    double ok_pos_tol_m{1e-3};
    double ok_rot_tol_rad{1e-2};
};

struct Ar5AnalyticalIkResult {
    bool ok{false};
    Eigen::VectorXd q;
    std::string diagnostic;
};

/// φ 局部扫描：score = −μ + w_lim·H + w_s·(Δφ)²（μ=Yoshikawa，H=限位裕量平方罚）
struct Ar5PhiScanSettings {
    double half_width{0.2};
    double step{0.01};
    double weight_limit{0.05};
    double weight_smooth{0.1};
    double analytic_max_pose_err{0.08};
};

struct Ar5PhiScanResult {
    bool ok{false};
    double phi{0.0};
    Eigen::VectorXd q_seed;
    double best_score{0.0};
    int num_candidates_evaluated{0};
};

/// 数值 IK 初值策略：Yoshikawa+Δφ²（原 phi_scan）或关节连续性的 ψ 扫描（离线测试同款打分）。
enum class Ar5NumericIkSeedPolicy {
    YoshikawaLocalPhi,
    JointContinuityPsi,
};

/**
 * 冗余臂数值 IK 迭代初值 q_init（YAML 与 ar5_analytical_ik 同级：redundant_numeric_ik_init）。
 * - LastIkOutput：上周期 IK 输出；无有效历史时回退 q_current
 * - CurrentMeasuredQ：始终用当前实测 q_current
 * - AnalyticTheory：半解析+扫描（参数见 ar5_analytical_ik）；hint/ref 优先上周期 IK，否则 q_current；失败回退 LastIkOutput 规则
 */
enum class RedundantNumericIkInit {
    LastIkOutput,
    CurrentMeasuredQ,
    AnalyticTheory,
};

/**
 * 关节连续性 ψ 扫描（与 scripts/ar5_analytical_ik_test_main 中 --psi-local-scan 一致）。
 * 在 pose_err_l2_log6 ≤ max_pose_l2_log6 的候选上最小化加权代价；可选 narrow_first / 全周。
 */
struct Ar5PsiContinuitySeedSettings {
    bool narrow_first{false};
    double narrow_half_width_rad{1.2};
    double narrow_step_rad{0.04};
    bool full_period{false};
    double half_width_rad{0.5};
    double step_rad{0.05};
    double max_pose_l2_log6{0.12};
    double score_tie_eps{0.02};
    double weight_joint_prev{1.0};
    double weight_joint_ref{0.0};
    double weight_inf_joint_prev{0.0};
    double weight_acc_second_diff{0.0};
    double weight_limit{0.05};
    double weight_psi_smooth{0.1};
    /// 仅当 state 尚未 phi_initialized 时用作 psi 初值（与独立调用本 API 时一致）
    double phi_init_rad{0.0};
    bool use_hint_as_joint_prev_when_no_history{true};
    bool clear_q_history_on_invalid_seed{true};
    /// 选到可行解后是否写回 state.phi_prev（随机 trial 每拍固定 ψ 时可设 false）
    bool update_phi_state_after_success{true};
    /// 选到可行 q_seed 后是否把 q 推入 q_tm1/q_tm2（供下一拍 w_acc / 连续性）
    bool advance_joint_history_from_seed{true};
};

/// 解析/数值 IK 初值链路的运行时状态（由控制节点或测试程序按周期持有）。
struct Ar5AnalyticalIkSeedRuntimeState {
    double phi_prev{0.0};
    bool phi_initialized{false};
    Eigen::VectorXd q_tm1;
    Eigen::VectorXd q_tm2;
    bool have_tm1{false};
    bool have_tm2{false};
};

/// 半解析/扫描参数；仅在 redundant_numeric_ik_init=analytic_theory 时由冗余 IK 步调用。
struct Ar5AnalyticalIkControlConfig {
    /// 首次 φ/ψ 状态 [rad]；之后由扫描结果递推（或由 JointContinuity 写回）
    double phi_init_rad{0.0};
    Ar5NumericIkSeedPolicy seed_policy{Ar5NumericIkSeedPolicy::YoshikawaLocalPhi};
    Ar5AnalyticalIkSettings analytic{};
    Ar5PhiScanSettings phi_scan{};
    Ar5PsiContinuitySeedSettings continuity_psi{};
    /// Yoshikawa 路径成功后是否同样维护 q_tm1/q_tm2（便于切换策略或外推）
    bool advance_joint_history_from_seed{true};
};

}  // namespace controllers
}  // namespace teleop_app
