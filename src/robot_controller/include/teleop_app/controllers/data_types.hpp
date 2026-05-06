#ifndef TELEOP_APP__CONTROLLERS__DATA_TYPES_HPP_
#define TELEOP_APP__CONTROLLERS__DATA_TYPES_HPP_

#include <vector>
#include <string>
#include <limits>
#include <cctype>
#include <cstdint>
#include <Eigen/Dense>
#include <map>

#include "teleop_app/controllers/ar5_analytical_ik_types.hpp"

namespace teleop_app {
namespace controllers {

/**
 * @brief 关节状态数据结构
 */
struct JointState {
    std::vector<double> position;     // 关节位置 (rad)
    std::vector<double> velocity;     // 关节速度 (rad/s)
    std::vector<double> torque;       // 关节力矩 (Nm)
    double timestamp;                 // 时间戳
};

/**
 * @brief 笛卡尔空间外力数据结构
 */
struct CartesianForce {
    double fx, fy, fz;                // 三维力 (N)
    double mx, my, mz;                // 三维力矩 (Nm)
    double gripper_force;            // 夹爪力 (N)
    double timestamp;                 // 时间戳
};

/**
 * @brief MIT 控制指令数据结构
 */
struct MitControlCommand {
    std::vector<double> position;     // 期望关节位置 (rad)
    std::vector<double> velocity;     // 期望关节速度 (rad/s)
    std::vector<double> feedforward_torque;  // 前馈力矩 (Nm)
    std::vector<double> kp;           // 刚度参数
    std::vector<double> kd;           // 阻尼参数
};

/**
 * @brief 运动规划结果数据结构
 */
struct MotionPlanningResult {
    MitControlCommand control_command;  // 本周期的控制指令
    bool target_reached;                // 目标位置是否达成
};

/**
 * @brief 夹爪状态数据结构
 */
 struct GripperState {
    // 关节状态
    std::vector<double> gripper_position;   // 夹爪关节位置 (rad for real robot, m for simulation)
    std::vector<double> gripper_velocity;   // 夹爪关节速度
    
    // 夹爪开合量（单位：mm）
    double opening_distance;     // 开合距离 (mm)
    
    // 旋转驱动电机角度（仅真实机器人，单位：rad）
    double motor_angle;          // 旋转电机角度 (rad, 仅真实机器人)
    
    // 时间戳
    double timestamp;
    
    GripperState()
        : opening_distance(0.0)
        , motor_angle(0.0)
        , timestamp(0.0) {
        gripper_position.resize(2, 0.0);
        gripper_velocity.resize(2, 0.0);
    }
};

/**
 * @brief 夹爪控制命令数据结构（使用GripperState）
 */
using GripperCommand = GripperState;

/**
 * @brief 机器人角色枚举
 */
enum class ArmRole {
    MASTER = 0,    // 主臂
    SLAVE = 1      // 从臂
};

/**
 * @brief 夹爪控制器配置
 */
struct GripperConfig {
    bool has_gripper;                   // 是否有夹爪
    std::vector<int> gripper_joint_indices;  // 夹爪关节在joint_indices中的索引（向量形式，兼容多指）
    std::string stroke_table_path;      // 查表文件路径（通用）
    std::string master_stroke_table_path; // 新增：主臂查表路径
    std::string slave_stroke_table_path;  // 新增：从臂查表路径
    std::string robot_type;             // 机器人类型
    // self_role - 当前这个 ArmController/GripperController 所控制的“本机夹爪”角色
    // 用途：updateState/本机夹爪状态解码时选择正确的查表（master/slave）与开合范围。
    // 注意：这不影响主从映射时对“从臂夹爪目标”的计算（那部分会显式传入 ArmRole::SLAVE）。
    std::string self_role;              // "master" / "slave"
    double max_opening_distance;         // 最大开合距离 (mm)
    double min_opening_distance;        // 最小开合距离 (mm)

    // 新增：主从臂开合距离范围 (mm)，用于归一化力反馈
    double master_max_opening_distance; // 主臂最大开合距离 (mm)
    double master_min_opening_distance; // 主臂最小开合距离 (mm)
    double slave_max_opening_distance;  // 从臂最大开合距离 (mm)
    double slave_min_opening_distance;  // 从臂最小开合距离 (mm)

    // MIT控制参数增益（1维，用于从臂夹爪控制）
    double kp;                          // 夹爪位置刚度
    double kd;                          // 夹爪速度阻尼
    
    // 力反馈参数（用于主从遥操作，独立于控制参数）
    double force_feedback_gain;        // 力反馈比例系数（0-1）
    double force_to_torque_scale;      // 力到力矩的转换系数（有效半径，m）
    bool force_feedback_use_fsm;       // 是否启用夹爪力反馈状态机
    double ff_x_on;                    // 进入夹持阈值（归一化差值）
    double ff_force_fadeout_range;     // 退出 HOLDING 后力反馈消退的主臂张开量（归一化）
    double ff_hold_entry_time_s;       // FREE->HOLDING 最小持续时间（秒）
    double ff_free_entry_time_s;       // HOLDING->FREE 最小持续时间（秒）
    double ff_free_master_slave_diff;   // 主臂开合度 > 从臂 + 该值时立即退出 HOLDING
    double ff_slave_velocity_threshold; // 从臂夹爪低速阈值（rad/s），进入 HOLDING 的额外条件
    double ff_reentry_cooldown_s;      // HOLDING->FREE 后禁止重新进入 HOLDING 的冷却时间（秒）
    double ff_exit_impedance_kp;      // 冷却期阻抗刚度（N），F = kp*(target-x) - kd*v
    double ff_exit_impedance_kd;      // 冷却期阻抗阻尼（N·s）
    double ff_ramp_tau_s;              // 进入 HOLDING 后从 lower 过渡到 upper 的时间常数（s）
    double ff_force_lower_limit;       // 力反馈下限（N，单边模型下建议 >= 0，作为“激活时最小反馈”）
    double ff_force_upper_limit;       // 力反馈上限（N，建议 > 0）

    // 主臂夹爪“回弹到最大开合度”的力矩前馈（单位：N·m）
    // 当不在最大开合度时，使用统一的定常力矩；接近完全弹开时，力矩线性下降至0
    double return_to_max_tau_constant;  // (N·m) 定常力矩大小，默认 0 表示关闭该行为
    double return_to_max_threshold;     // 归一化开合度阈值，距离完全张开还剩该值时开始线性下降（例如 0.02）
    double return_to_max_close_velocity_threshold; // (rad/s) 闭合方向速度阈值：仅当 v < -th 时禁用回弹
    double ff_holding_gripper_kd;                   // 力反馈非零时主臂夹爪额外阻尼，直接加到 cmd.kd

    // 新增：夹爪平滑参数
    double target_angle_filter_alpha;   // 目标角度滤波系数
    double max_gripper_delta;           // 最大允许角度差值 (rad)
    // 新增：仅在低速时启用 max_gripper_delta 限幅（rad/s）。默认给一个很大的值，相当于“始终启用限幅”。
    double max_gripper_delta_velocity_threshold;
    double kd_velocity_limit;           // kd 阻尼项的速度上限（rad/s），超过此值按此值计算阻尼
    
    GripperConfig()
        : has_gripper(false)
        , self_role("slave")
        , max_opening_distance(100.0)
        , min_opening_distance(0.0)
        , master_max_opening_distance(30.0)
        , master_min_opening_distance(0.0)
        , slave_max_opening_distance(100.0)
        , slave_min_opening_distance(0.0)
        , kp(0.0)
        , kd(0.1)
        , force_feedback_gain(1.0)
        , force_to_torque_scale(0.03)
        , force_feedback_use_fsm(true)
        , ff_x_on(0.30)
        , ff_force_fadeout_range(0.10)
        , ff_hold_entry_time_s(0.08)
        , ff_free_entry_time_s(0.12)
        , ff_free_master_slave_diff(0.1)
        , ff_slave_velocity_threshold(0.2)
        , ff_reentry_cooldown_s(0.5)
        , ff_exit_impedance_kp(5.0)
        , ff_exit_impedance_kd(0.5)
        , ff_ramp_tau_s(0.05)
        , ff_force_lower_limit(0.0)
        , ff_force_upper_limit(0.2)
        , return_to_max_tau_constant(0.0)
        , return_to_max_threshold(0.02)
        , return_to_max_close_velocity_threshold(0.0)
        , ff_holding_gripper_kd(0.0)
        , target_angle_filter_alpha(1.0)
        , max_gripper_delta(0.5)
        , max_gripper_delta_velocity_threshold(std::numeric_limits<double>::infinity())
        , kd_velocity_limit(std::numeric_limits<double>::infinity()) {
        gripper_joint_indices.clear();
    }
};

/**
 * @brief 关节限制参数
 */
struct JointLimits {
    std::vector<double> max_position;      // 最大关节位置 (rad)
    std::vector<double> min_position;      // 最小关节位置 (rad)
    std::vector<double> max_velocity;      // 最大速度 (rad/s)
    std::vector<double> max_acceleration;  // 最大加速度 (rad/s²)
    std::vector<double> max_jerk;          // 最大加加速度 (rad/s³)
    
    JointLimits() {
        max_position.clear();
        min_position.clear();
        max_velocity.clear();
        max_acceleration.clear();
        max_jerk.clear();
    }
};

/**
 * @brief 电机参数
 */
struct MotorParams {
    std::vector<double> rotor_inertia;     // 电机转子惯量 (kg·m²)
    std::vector<double> gear_ratio;        // 减速比
    
    MotorParams() {
        rotor_inertia.clear();
        gear_ratio.clear();
    }
};

/**
 * @brief 运动规划类型枚举
 */
enum class MotionPlanningType {
    INTERPOLATE = 0,  // 线性插值规划
    SCURVE = 1,       // S曲线规划
    POLYNOMIAL = 2    // 多项式插值规划（五次多项式）
};

/**
 * @brief 逆运动学数值内核：与 YAML ik_config.ik_algorithm 字符串对应。
 *        - ProjectedDampedLeastSquares：阻尼广义逆 + 零空间投影（原有实现）。
 *        - QpWeightedLeastSquaresTaskSlack：单层凸 QP，变量 [Δq; s]，加权末端残差 + 任务松弛 ‖s‖²。
 *        - VelocityLevelQp：速度级 QP + CLIK，变量 [q̇; s]，关节速度盒 + 位置速度化约束。
 */
enum class IkAlgorithm : std::uint8_t {
    ProjectedDampedLeastSquares = 0,
    QpWeightedLeastSquaresTaskSlack = 1,
    VelocityLevelQp = 2,
};

/// QP-WLS 零空间参考关节策略：决定 q_ref 的来源
enum class QpRefJointPolicy : std::uint8_t {
    LastQTarget = 0,     ///< 上周期 q_target（默认，连续性好）
    JointMidpoint = 1,   ///< 关节活动范围中点（主动远离极限/常见奇异位型）
    UserSpecified = 2,   ///< 用户指定固定姿态
};

/**
 * @brief 逆运动学求解器配置结构体
 */
 struct IKConfig {
    /// YAML: ik_algorithm，默认投影 DLS；可选 qp_wls_task_slack
    IkAlgorithm ik_algorithm = IkAlgorithm::ProjectedDampedLeastSquares;

    double ik_damp;                    // 阻尼系数
    Eigen::Matrix<double, 6, 1> weight_ik;  // 位姿权重
    int ik_iter_max;                   // 最大迭代次数
    double ik_dt;                      // 时间步长
    double singular_values_min;        // 最小奇异值阈值
    double max_damp;                   // 最大阻尼系数
    double epsilon;                    // 收敛阈值
    // 零空间基准增益 k_ns = ref_joint_penalty * ref_joint_kp；实际 w = k_ns * α * (q_ref - q)
    // α = α_err * α_σ，见 RobotModel（大末端误差或近奇异时 α→0，优先 IK 收敛）
    double ref_joint_penalty;
    double ref_joint_kp;
    // 加权 ‖W e‖ 超过门限时 α_err=0；≤0 时用 max(12*epsilon, 0.04)
    double nullspace_error_gate;
    // ‖Δq_ns‖ 相对 ‖Δq_task‖ 上限；≤0 表示不限制
    double nullspace_dq_max_ratio;
    // 零空间可操作度：w += k_m · β_err · β_σ · ∇m（m 来自 det(JJᵀ+εI)）；k_m=0 关闭
    double nullspace_manip_gain;
    double nullspace_manip_fd_step;
    double nullspace_manip_det_eps;
    double nullspace_manip_grad_clip;
    // β_σ 用 (σ_ref−σ_min)/σ_ref；≤0 时 σ_ref=max(3·singular_values_min, 0.05)
    double nullspace_manip_sigma_ref;
    // β_err = pow(α_err, p)，p=1 与参考关节项同门控；p>1 更晚介入
    double nullspace_manip_err_gate_power;

    // -------- 仅 QpWeightedLeastSquaresTaskSlack 使用（投影 DLS 忽略）--------
    /// 惩罚 ‖s‖²，越大越不愿用松弛（越接近硬约束末端等式）
    double qp_slack_penalty = 1000.0;
    /// Tikhonov：在 ‖Δq‖² 上，抑奇异与大步
    double qp_tikhonov_delta_q = 1e-4;
    /// 软跟踪可操作度偏置 ‖Δq - Δq_ns‖²，无关节姿态参考项
    double qp_nullspace_weight = 0.02;
    bool qp_manipulability_enable = true;
    double qp_manipulability_gain = 0.035;
    double qp_manipulability_fd_step = 8e-5;
    double qp_manipulability_grad_clip = 30.0;
    double qp_manipulability_det_eps = 1e-6;
    /// 上界：‖W e‖ 须 **低于** 该值才可能掺入 Δq_ns（≤0 时用 max(12*epsilon, 0.04)）
    double qp_nullspace_error_gate = 0.10;
    /// 下界：‖W e‖ 还须 **高于** 该值才掺入；否则视为「已贴目标」不爬可操作度（≤0 时用 max(3*epsilon, 5e-3)）
    double qp_nullspace_error_min = 0.0;
    /// 还要求 σ_min(J) 大于该值才掺入可操作度引导
    double qp_nullspace_sigma_min_gate = 0.01;

    // -------- QP-WLS 零空间参考关节位控制（将冗余自由度拉向 q_ref，防漂移）--------
    /// 启用后在 QP 目标函数中增加 w_ref·‖Δq − α(q_ref−q)‖²
    bool qp_ref_joint_enable = false;
    /// 参考关节项权重；↑ 冗余自由度回复力更强，↓ 更不影响末端跟踪
    double qp_ref_joint_weight = 0.02;
    /// 误差门控上界：‖W e‖ 低于此值才启用 q_ref 回复（≤0 时用 max(12*epsilon, 0.04)）
    double qp_ref_joint_error_gate = 0.10;
    /// 奇异值门控：σ_min < 此值时衰减 q_ref 增益（避免奇异点附近零空间项与任务项冲突）
    double qp_ref_joint_sigma_gate = 0.01;

    // -------- QP-WLS 自适应 Tikhonov 正则化（奇异点附近增加 ‖Δq‖² 阻尼）--------
    /// 启用后 w_reg = qp_tikhonov_delta_q + boost_max·ratio²，ratio = (1 − σ_min/σ_threshold)
    bool qp_adaptive_tikhonov_enable = false;
    /// σ_min 低于此阈值时开始增加正则化
    double qp_adaptive_tikhonov_sigma_threshold = 0.01;
    /// 最大额外正则化增量（奇异点正中心时叠加的峰值）
    double qp_adaptive_tikhonov_max_boost = 0.01;

    // -------- QP-WLS 单周期关节变化量裁剪 --------
    /// IK 输出 q_out 相对初值 q_init 的最大 L2 范数；≤0 禁用
    double qp_max_total_joint_change = 0.0;

    // -------- QP-WLS 可操作度 CBF 不等式约束（硬边界防止进入低可操作度区域）--------
    /// 启用后在 QP 约束中增加 ik_dt·∇μᵀΔq ≥ γ(μ_min − μ)
    bool qp_manip_cbf_enable = false;
    /// 可操作度下限阈值；需根据臂型标定（Yoshikawa μ 量级）
    double qp_manip_cbf_mu_min = 0.01;
    /// CBF 增益 γ ∈ (0,1]；越大越积极远离低可操作度边界
    double qp_manip_cbf_gamma = 0.5;

    // -------- QP-WLS 零空间参考关节策略 --------
    /// q_ref 来源策略
    QpRefJointPolicy qp_ref_joint_policy = QpRefJointPolicy::LastQTarget;
    /// 策略为 UserSpecified 时的固定参考关节位（rad）；size 须等于 nv
    std::vector<double> qp_ref_joint_user_specified{};

    // ======== 仅 VelocityLevelQp 使用（其他算法忽略） ========

    // CLIK 比例增益：ẋ_des = diag(vqp_clik_gain) · log6(T_des⁻¹ T_ee)
    Eigen::Matrix<double, 6, 1> vqp_clik_gain =
        (Eigen::Matrix<double, 6, 1>() << 5.0, 5.0, 5.0, 3.0, 3.0, 3.0).finished();

    /// 任务松弛惩罚 ρ
    double vqp_slack_penalty = 10000.0;
    /// 基础正则化 λ·‖q̇‖²
    double vqp_regularization = 2.5e-4;

    // 自适应正则化（奇异点附近 λ 自动增大）
    bool   vqp_adaptive_reg_enable = true;
    double vqp_adaptive_reg_sigma_threshold = 0.015;
    double vqp_adaptive_reg_max_boost = 0.008;

    /// 关节速度盒约束 (rad/s)，每关节独立；空则不启用
    std::vector<double> vqp_joint_velocity_limits{};

    /// 位置约束速度化增益 k：q̇ ∈ [k(q_min−q), k(q_max−q)]；≤0 禁用
    double vqp_position_limit_gain = 10.0;

    /// 末端速度限制 [vx,vy,vz,wx,wy,wz] (m/s, rad/s)；空 = 不限
    std::vector<double> vqp_ee_velocity_limits{};

    // 可操作度梯度注入目标函数（软引导远离奇异）
    bool   vqp_manipulability_grad_enable = false;
    double vqp_manipulability_grad_gain = 0.02;
    double vqp_manipulability_fd_step = 8e-5;
    double vqp_manipulability_grad_clip = 25.0;
    double vqp_manipulability_det_eps = 1e-6;

    // Barrier 函数 β/w(q)（越近奇异惩罚越大）
    bool   vqp_barrier_enable = false;
    double vqp_barrier_gain = 0.01;
    double vqp_barrier_w_min_clamp = 1e-4;

    // 可操作度 CBF 硬约束
    bool   vqp_cbf_enable = true;
    double vqp_cbf_mu_min = 0.008;
    double vqp_cbf_gamma = 0.5;

    // 零空间参考关节
    bool   vqp_ref_joint_enable = true;
    double vqp_ref_joint_weight = 0.003;
    double vqp_ref_joint_error_gate = 0.10;
    double vqp_ref_joint_sigma_gate = 0.015;
    /// q_ref 来源策略（复用 QpRefJointPolicy 枚举）
    QpRefJointPolicy vqp_ref_joint_policy = QpRefJointPolicy::LastQTarget;
    std::vector<double> vqp_ref_joint_user_specified{};

    /// 零空间可操作度梯度偏置（目标函数中 w_ns·‖q̇ − q̇_ns‖²）
    double vqp_nullspace_weight = 0.0;

    // Arm-angle / elbow swivel 控制：约束肩-肘-腕平面，抑制冗余臂角翻转
    bool vqp_arm_angle_enable = false;
    double vqp_arm_angle_weight = 0.03;
    double vqp_arm_angle_kp = 1.5;
    std::string vqp_arm_angle_ref_policy = "q_ref";  // q_ref / initial / user_specified
    std::vector<double> vqp_arm_angle_ref_joint_positions{};
    double vqp_arm_angle_user_ref = 0.0;
    bool vqp_arm_angle_limit_enable = false;
    double vqp_arm_angle_min = -0.7;
    double vqp_arm_angle_max = 0.7;
    double vqp_arm_angle_limit_gain = 5.0;
    double vqp_arm_angle_fd_step = 1e-4;
    std::string vqp_arm_angle_shoulder_frame = "AR5-5_07L-W4C4A2_link2";
    std::string vqp_arm_angle_elbow_frame = "AR5-5_07L-W4C4A2_link4";
    std::string vqp_arm_angle_wrist_frame = "AR5-5_07L-W4C4A2_link7";

    /// 单周期 ‖q̇‖ 上限 (rad/s)；≤0 不限
    double vqp_max_qdot_norm = 0.0;

    /// 单周期关节变化量 ‖q_out − q_init‖ 上限 (rad)；≤0 不限
    double vqp_max_total_joint_change = 1.0;

    /// 当 FK(当前 q) 相对目标的加权末端误差 ‖W·log6(T_des⁻¹T_ee)‖ 足够小时，**不调用**数值 IK，直接保持当前关节（抑制冗余零空间蠕动；仿真重力+位控时尤明显）
    bool ik_hold_skip_solver_when_within_tolerance = false;
    /// 跳过 IK 的误差上限；≤0 时用 epsilon（与收敛判据同量级）
    double ik_hold_skip_weighted_norm_max = 0.0;

    // 构造函数
    IKConfig() 
        : ik_damp(0.01),
          weight_ik(Eigen::Matrix<double, 6, 1>::Ones()),
          ik_iter_max(50),
          ik_dt(1.0),
          singular_values_min(1e-6),
          max_damp(0.1),
          epsilon(1e-6),
          ref_joint_penalty(0.01),
          ref_joint_kp(1.0),
          nullspace_error_gate(0.0),
          nullspace_dq_max_ratio(1.0),
          nullspace_manip_gain(0.0),
          nullspace_manip_fd_step(1e-5),
          nullspace_manip_det_eps(1e-6),
          nullspace_manip_grad_clip(80.0),
          nullspace_manip_sigma_ref(0.0),
          nullspace_manip_err_gate_power(2.0),
          qp_slack_penalty(1000.0),
          qp_tikhonov_delta_q(1e-4),
          qp_nullspace_weight(0.02),
          qp_manipulability_enable(true),
          qp_manipulability_gain(0.035),
          qp_manipulability_fd_step(8e-5),
          qp_manipulability_grad_clip(30.0),
          qp_manipulability_det_eps(1e-6),
          qp_nullspace_error_gate(0.10),
          qp_nullspace_error_min(0.0),
          qp_nullspace_sigma_min_gate(0.01),
          qp_ref_joint_enable(false),
          qp_ref_joint_weight(0.02),
          qp_ref_joint_error_gate(0.10),
          qp_ref_joint_sigma_gate(0.01),
          qp_adaptive_tikhonov_enable(false),
          qp_adaptive_tikhonov_sigma_threshold(0.01),
          qp_adaptive_tikhonov_max_boost(0.01),
          qp_max_total_joint_change(0.0),
          qp_manip_cbf_enable(false),
          qp_manip_cbf_mu_min(0.01),
          qp_manip_cbf_gamma(0.5),
          qp_ref_joint_policy(QpRefJointPolicy::LastQTarget),
          qp_ref_joint_user_specified(),
          vqp_clik_gain((Eigen::Matrix<double, 6, 1>() << 5.0, 5.0, 5.0, 3.0, 3.0, 3.0).finished()),
          vqp_slack_penalty(10000.0),
          vqp_regularization(2.5e-4),
          vqp_adaptive_reg_enable(true),
          vqp_adaptive_reg_sigma_threshold(0.015),
          vqp_adaptive_reg_max_boost(0.008),
          vqp_joint_velocity_limits(),
          vqp_position_limit_gain(10.0),
          vqp_ee_velocity_limits(),
          vqp_manipulability_grad_enable(false),
          vqp_manipulability_grad_gain(0.02),
          vqp_manipulability_fd_step(8e-5),
          vqp_manipulability_grad_clip(25.0),
          vqp_manipulability_det_eps(1e-6),
          vqp_barrier_enable(false),
          vqp_barrier_gain(0.01),
          vqp_barrier_w_min_clamp(1e-4),
          vqp_cbf_enable(true),
          vqp_cbf_mu_min(0.008),
          vqp_cbf_gamma(0.5),
          vqp_ref_joint_enable(true),
          vqp_ref_joint_weight(0.003),
          vqp_ref_joint_error_gate(0.10),
          vqp_ref_joint_sigma_gate(0.015),
          vqp_ref_joint_policy(QpRefJointPolicy::LastQTarget),
          vqp_ref_joint_user_specified(),
          vqp_nullspace_weight(0.0),
          vqp_arm_angle_enable(false),
          vqp_arm_angle_weight(0.03),
          vqp_arm_angle_kp(1.5),
          vqp_arm_angle_ref_policy("q_ref"),
          vqp_arm_angle_ref_joint_positions(),
          vqp_arm_angle_user_ref(0.0),
          vqp_arm_angle_limit_enable(false),
          vqp_arm_angle_min(-0.7),
          vqp_arm_angle_max(0.7),
          vqp_arm_angle_limit_gain(5.0),
          vqp_arm_angle_fd_step(1e-4),
          vqp_arm_angle_shoulder_frame("AR5-5_07L-W4C4A2_link2"),
          vqp_arm_angle_elbow_frame("AR5-5_07L-W4C4A2_link4"),
          vqp_arm_angle_wrist_frame("AR5-5_07L-W4C4A2_link7"),
          vqp_max_qdot_norm(0.0),
          vqp_max_total_joint_change(1.0),
          ik_hold_skip_solver_when_within_tolerance(false),
          ik_hold_skip_weighted_norm_max(0.0) {}
};

/**
 * @param s 小写去空格后匹配；未知值回退为投影 DLS。
 */
inline IkAlgorithm ikAlgorithmFromRosParamString(const std::string& s) {
    std::string t;
    t.reserve(s.size());
    for (char c : s) {
        if (c != ' ' && c != '\t') {
            t.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }
    }
    if (t == "qp_wls_task_slack" || t == "qp-weighted-least-squares-task-slack") {
        return IkAlgorithm::QpWeightedLeastSquaresTaskSlack;
    }
    if (t == "velocity_level_qp" || t == "velocity-level-qp" || t == "vqp") {
        return IkAlgorithm::VelocityLevelQp;
    }
    return IkAlgorithm::ProjectedDampedLeastSquares;
}

/**
 * @brief 冗余臂（arm_dof>6）IK 链路安全：进 IK 前裁剪末端位置、IK 迭代内关节夹紧。
 *        与 cbf_config 无关；不依赖 enable_cbf。
 */
struct RedundantIkSafetyConfig {
    bool enable_task_space_box = false;
    struct TaskSpaceBox {
        std::map<std::string, std::pair<double, double>> limits;
        double safety_margin = 0.02;
    } task_space;
    bool enable_joint_limit_clamp = false;
    struct JointLimitClamp {
        std::vector<double> min;
        std::vector<double> max;
        double safety_margin = 0.05;
    } joint_limit;
};

/// 平滑下发 MIT 的臂关节 cmd.position/cmd.velocity；不影响 CBF 内部 q_target、v_target。
struct PostCbfJointReferenceFilterConfig {
    bool enable = false;
    /// 对 q_cbf 滑动均值窗口（拍），再送入低通；1 表示不做均值
    int position_mean_window = 1;
    /// cmd 位置低通：q_f ← (1−a)·q_f + a·q_ma；a∈(0,1]；q_ma 为当前窗口内 q_cbf 的算术均值
    double position_new_weight = 0.2;
    /// cmd 速度低通：v_f ← (1−a)·v_f + a·diff(q_f)/dt；a∈(0,1]
    double velocity_new_weight = 0.2;
};

/**
 * @brief 动力学外力估计器配置（AR5 并列路径，默认关闭）
 *
 * 对应 YAML 节点 `dynamics_force_estimator:` 及顶层字段
 * `force_estimation_method: virtual_spring / quasi_static /
 *  momentum_observer / inv_dyn_residual`。
 *
 * 与现有虚拟弹簧路径（ee_ff_*）并列，通过 force_estimation_method
 * 选择其中一条，互不影响。
 */
struct Ar5DynamicsForceEstimatorConfig {
    /// 方法名：virtual_spring(默认) / quasi_static /
    ///          momentum_observer / inv_dyn_residual / delta_momentum
    std::string method{"virtual_spring"};

    /// 惯量矩阵选择：false→纯连杆 CRBA；true→含电机折合惯量
    /// 实机调试时建议设为 true
    bool use_motor_inertia{false};

    /// 阻尼伪逆正则化因子 λ（雅可比奇异时防止数值爆炸）
    /// 越大近奇异越稳定，越小力估计精度越高；典型值 0.005~0.05
    double damped_pinv_lambda{0.01};

    /// 动量观测器增益 K_obs（标量，乘单位阵）
    /// 越大跟踪越快但对噪声越敏感；典型值 5~30
    double momentum_observer_gain{10.0};

    /// 差分动量观测器残差 IIR 滤波系数（仅 delta_momentum 路径）
    /// 越大跟踪越快但噪声越大；典型值 0.05~0.3
    double delta_momentum_alpha{0.1};

    /// 逆动力学残差路径：速度一阶 IIR 预滤波系数
    /// 滤波后速度用于数值差分求加速度和 RNEA 计算
    /// 1.0=不滤波（直通），越小越平滑；典型值 0.5~1.0
    double velocity_filter_alpha{1.0};

    /// 逆动力学残差路径：加速度数值微分一阶 IIR 系数
    /// 越大越跟踪原始加速度，越小越平滑；典型值 0.1~0.5
    double accel_filter_alpha{0.3};

    /// 输出 6D 力一阶 IIR 低通系数（由截止频率和 dt 计算得到）
    /// 1.0=不滤波；越小越平滑但延迟越大
    double output_filter_alpha{1.0};

    /// 输出低通截止频率 (Hz)，0=不滤波
    /// 从中计算 output_filter_alpha = ωdt/(ωdt+1)
    double output_filter_cutoff_hz{5.0};

    /// 控制周期 (s)，由上层 forcePublishThread 频率决定（通常 ~0.01s）
    double dt{0.01};

    /// 简单静摩擦补偿：从关节实测力矩中减去该偏置（N·m，7维）
    /// 仅在 quasi_static / inv_dyn_residual 路径生效
    bool enable_friction_offset{false};
    std::vector<double> friction_torque_offset{};
};

/**
 * @brief 控制器参数配置
 */
struct ControllerParams {
    std::string urdf_path;              // URDF 文件路径
    std::string cbf_urdf_path;          // CBF 专用的 URDF 文件路径（如果为空则使用 urdf_path）
    std::string robot_type;             // 机型标识（例如: "nexus", "y1"），用于机型相关策略（如仅 Nexus 启用摩擦补偿）
    std::string arm_base_frame;         // 支链根坐标系（为空则使用完整模型）
    std::string end_effector_frame;     // 支链末端坐标系
    std::vector<int> joint_indices;     // 关节索引（用于子树提取）
    std::vector<std::string> joint_names;  // 关节名称（与 joint_indices 一一对应，用于验证）
    
    // 仿真模式标志
    bool is_simulation;                 // 是否为仿真模式（true: 不加电机惯量, false: 加电机惯量）
    
    // 关节限制（仅针对arm的arm_dof个关节）
    JointLimits joint_limits;
    // 电机参数（仅针对arm的arm_dof个关节）
    MotorParams motor_params;
    
    // 夹爪配置
    GripperConfig gripper_config;
    
    // 逆运动学求解器配置
    IKConfig ik_config;             // IK配置参数
    /// 冗余臂 IK 侧工作空间盒与关节夹紧（YAML：与 ik_config 同级 redundant_ik_safety）
    RedundantIkSafetyConfig redundant_ik_safety;
    /// 冗余 7 轴数值 IK 迭代初值（YAML：与 ar5_analytical_ik 同级 redundant_numeric_ik_init）
    RedundantNumericIkInit redundant_numeric_ik_init{RedundantNumericIkInit::LastIkOutput};
    /// AR5：半解析 IK + 扫描参数；仅当 redundant_numeric_ik_init=analytic_theory 时参与初值
    Ar5AnalyticalIkControlConfig ar5_analytical_ik{};
    /// 仅 MIT 臂关节 cmd 位置/速度：位置滤波，速度由滤波位置差分并低通；不关 CBF 状态
    PostCbfJointReferenceFilterConfig post_cbf_joint_reference_filter{};
    /// MIT cmd.velocity 置零（true 时不使用 CBF 输出的速度，cmd.velocity 全部为 0）
    bool mit_cmd_velocity_zero{false};

    // 控制周期时间步长（秒）
    double ctrl_dt;                     // 控制周期时间步长，根据 control_publish_rate 计算
    
    // 运动规划类型
    MotionPlanningType motion_planning_type;  // 运动规划类型：INTERPOLATE 或 SCURVE
    
    // 末端力反馈（6D）：弹簧阻尼模型 + 位姿偏差死区 + 速度门控
    std::vector<double> ee_ff_delta_threshold;      // [6] 位姿偏差死区：|δx(i)| < threshold 时该轴不计算力反馈
    std::vector<double> ee_ff_delta_hold_time;     // [6] 各轴偏差持续超阈值多久后才触发力反馈 (s)
    std::vector<double> ee_ff_velocity_threshold;   // [6] 各轴末端速度门控阈值，|v_ee_local(i)| ≥ threshold 时不计算力反馈
    std::vector<double> ee_ff_spring_kp;            // [6] 弹簧刚度 (N/m, N·m/rad)
    std::vector<double> ee_ff_damper_kd;            // [6] 阻尼系数 (N·s/m, N·m·s/rad)
    std::vector<double> ee_ff_holding_kd;           // [6] 额外人工阻尼 (N·s/m, N·m·s/rad)
    std::vector<double> ee_ff_force_lower_limit;    // [6] 各轴总力反馈输出死区：|F| < lower → F=0
    std::vector<double> ee_ff_force_upper_limit;    // [6] 各轴总力反馈饱和上限
    // 力反馈发布侧统一保护（所有估计路径共用，在发布前生效）
    std::vector<double> force_publish_lower_limit;  // [6] 死区：|F(i)| < lower → 0 (N, N·m)
    std::vector<double> force_publish_upper_limit;  // [6] 饱和：clamp to ±upper (N, N·m)
    double force_publish_velocity_gate{0.0};        // 关节速度范数门控 (rad/s)：‖v‖ ≥ gate 时力置零；0=不启用
    double ee_ff_gain{0.0};                             // 主臂：末端力反馈增益，0=关闭
    std::vector<double> ee_ff_static_friction_margin; // 主臂：每关节静摩擦抬升裕量(N·m)，长度=arm_dof
    bool enable_ee_ff_static_friction_lift{false};     // 主臂：是否启用原有“力反馈抬升过静摩擦”逻辑，默认关闭

    // 自由度
    int arm_dof;    // 机械臂自由度
    int gripper_jnt_num;    // 夹爪关节数量
    int total_dof;    // arm_dof + 1

    // 重力补偿控制参数（arm_dof个）
    std::vector<double> gravity_comp_kp;    // 重力补偿 Kp - 通常较小，允许拖动
    std::vector<double> gravity_comp_kd;    // 重力补偿 Kd - 提供阻尼
    
    // 运动规划控制参数（arm_dof + 1个，用于夹爪）
    std::vector<double> motion_planning_kp; // 运动规划 Kp - 通常较大，快速跟踪
    std::vector<double> motion_planning_kd; // 运动规划 Kd - 避免超调
    /// MIT 关节指令 cmd.kd 上额外叠加的人工阻尼（每关节，长度 arm_dof；缺省全 0）
    std::vector<double> mit_artificial_kd{};

    // 阻尼控制参数（用于 IDLE 和 FAULT 状态）
    std::vector<double> damping_control_kp; // 阻尼控制 Kp (每个关节) - 用于空闲和故障状态
    std::vector<double> damping_control_kd; // 阻尼控制 Kd (每个关节) - 用于空闲和故障状态

    // 笛卡尔空间阻抗（6D：[x,y,z,roll,pitch,yaw]）；空则 arm_controller 用内置默认（如 AR5 不写 YAML）
    std::vector<double> cart_imped_kp;
    std::vector<double> cart_imped_kd;
    // 6 轴力矩阶 CBF 名义笛卡尔阻抗；空则 arm_controller 用内置默认
    std::vector<double> cbf_cart_kp;
    std::vector<double> cbf_cart_zeta;
    /// 历史字段，控制器未使用；保留加载以兼容旧 YAML
    std::vector<double> cbf_inner_joint_impedance_kp{};
    std::vector<double> cbf_inner_joint_impedance_kd{};

    // 对角化阻尼配置
    bool enable_double_diag_damping;  // 是否启用双对角化阻尼（获得常定阻尼比动态）
    
    // PID积分控制参数（用于消除静态误差）
    bool enable_integral_control;           // 是否启用积分控制
    std::vector<double> joint_ki;           // 关节空间积分增益（arm_dof个）
    std::vector<double> integral_limit;     // 积分力矩限幅（N·m，arm_dof个）
    std::vector<double> output_limit;  // PID总输出力矩限幅（N·m，arm_dof个）

    // 新增：摩擦补偿和自适应Kp参数
    // 注意：这些字段必须有安全默认值/默认行为，否则在未从 YAML/参数服务器加载时会导致力矩计算异常（抖动/坠落）。
    double friction_velocity_threshold{0.04};             // 摩擦补偿速度阈值 (rad/s)
    int velocity_sign_history_size{10};                   // 符号历史队列大小
    double friction_tanh_velocity_scale{1.0};             // tanh过渡速度尺度系数：scale 越大过渡越快（tanh(|v| * scale / v_th)）
    std::vector<double> friction_compensation_gain{};     // 每关节摩擦补偿增益（长度=arm_dof，空=全0）
    double friction_compensation_cartesian_error_threshold_full{0.01}; // 笛卡尔全补偿误差阈值 (m)
    double friction_compensation_cartesian_error_threshold_zero{0.002}; // 笛卡尔零补偿误差阈值 (m)
    // 静摩擦方波补偿：当不满足动摩擦补偿条件时视为静止，施加高频正负方波，幅值 = F_c * amplitude_scale（每关节）
    std::vector<double> static_friction_compensation_amplitude_scale{};  // 每关节方波幅值系数（长度=arm_dof）
    std::vector<double> static_friction_compensation_frequency{};       // 每关节方波频率 (Hz)（长度=arm_dof）
    double gravity_comp_kp_velocity_threshold{0.05};      // 自适应Kp速度阈值 (rad/s)

    // 关节软限位（FSM 模式：FREE↔HOLDING，带迟滞防抖和时间 ramp；仅针对 arm_dof 个关节）
    bool enable_joint_soft_limit{false};
    std::vector<double> soft_limit_lower{};
    std::vector<double> soft_limit_upper{};
    std::vector<double> soft_limit_kp{};
    std::vector<double> soft_limit_kd{};
    std::vector<double> soft_limit_transition_width{};
    std::vector<double> soft_limit_tau_max{};
    std::vector<int> soft_limit_unidirectional_damping{};
    
    bool enable_cbf = false;  // 是否启用 CBF
    std::string robot_name;  // 机器人名称
    std::vector<int> arm_joint_indices;  // 机械臂关节索引（用于CBF，不包含夹爪）
    struct CBFConfig {
        bool enable_task_space = false;
        bool enable_joint_limit = false;
        bool enable_joint_velocity = false;
        bool enable_self_collision = false;
        bool enable_arm_obstacle_avoidance = false;  // 是否启用整臂避障（替代末端避障）
        
        // 碰撞对列表（用于自碰撞）
        std::vector<std::pair<std::string, std::string>> collision_pairs;
        
        // 任务空间配置
        struct TaskSpaceConfig {
            std::map<std::string, std::pair<double, double>> limits;
            double alpha = 100.0;
            double safety_margin = 0.01;
            /** 速度QP：h 大于该内侧距离(米)则 CBF RHS 放松为极负，避免远离包络仍被 ḣ+αh 夹死。<-1 或未配置时用自动裕量 */
            double interior_cbf_deactivate_margin = -1.0;
        } task_space_config;
        
        // 关节限位配置
        struct JointLimitConfig {
            std::vector<double> min;
            std::vector<double> max;
            double safety_margin = 0.01;
            double alpha = 200.0;
            /** 速度QP：距限位内侧超过该值(rad)则放松 RHS；<-1 自动；=0 始终用完整 -αh（最严） */
            double interior_cbf_deactivate_margin = -1.0;
        } joint_limit_config;
        
        // 关节速度约束配置
        struct JointVelocityConfig {
            std::vector<double> min;           // 最小速度限制 (rad/s)
            std::vector<double> max;           // 最大速度限制 (rad/s)
            double safety_margin = 0.1;        // 安全边距 (rad/s)
            double alpha = 10.0;               // CBF 系数
        } joint_velocity_config;
        
        // 自碰撞配置
        struct SelfCollisionConfig {
            double safety_distance = 0.02;
            double alpha = 100.0;
        } self_collision_config;
        
        // 默认障碍物配置
        struct ObstacleConfig {
            double safety_distance = 0.1;
            double obstacle_radius = 0.03;
            double ee_radius = 0.03;
            double alpha = 10.0;
        } default_obstacle_config;
    } cbf_config;

    /// 动力学外力估计器（与虚拟弹簧并列，默认关闭）
    Ar5DynamicsForceEstimatorConfig dynamics_force_estimator{};
};

/**
 * @brief 控制器状态
 */
enum class ControllerStatus {
    UNINITIALIZED,      // 未初始化
    INITIALIZED,        // 已初始化
    RUNNING,            // 运行中
    ERROR               // 错误状态
};



}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__DATA_TYPES_HPP_
