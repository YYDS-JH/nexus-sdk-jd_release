#ifndef TELEOP_APP__CONTROLLERS__ROBOT_MODEL_HPP_
#define TELEOP_APP__CONTROLLERS__ROBOT_MODEL_HPP_

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include "teleop_app/controllers/data_types.hpp"

namespace teleop_app {
namespace controllers {

/**
 * @brief 机器人运动学/动力学模型封装类
 * 
 * 封装 Pinocchio 库，提供：
 * - 完整模型和子树模型的加载
 * - 正/逆运动学计算
 * - 动力学计算（重力、科氏力、惯性矩阵）
 * - 雅可比矩阵计算
 */
class RobotModel {
public:
    /**
     * @brief 模型配置参数
     */
    struct Config {
        std::string urdf_path;              // URDF 文件路径
        std::string base_frame;             // 支链根坐标系（为空则使用完整模型）
        std::string end_effector_frame;     // 支链末端坐标系
        std::vector<int> joint_indices;     // 关节索引（用于子树提取）
        std::vector<double> motor_rotor_inertia;  // 电机转子惯量 (kg·m²)
        std::vector<double> motor_gear_ratio;     // 减速比
    };

    RobotModel();
    ~RobotModel();
    RobotModel(RobotModel&&) noexcept;
    RobotModel& operator=(RobotModel&&) noexcept;

    /**
     * @brief 初始化模型
     * 
     * @param config 模型配置参数
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool initialize(const Config& config);

    /**
     * @brief 检查模型是否已加载
     */
    bool isLoaded() const;

    /**
     * @brief 获取模型自由度数量
     */
    int getDOF() const;

    /**
     * @brief 获取电机转子惯量配置（电机轴侧，kg·m²）
     */
    const std::vector<double>& getMotorRotorInertia() const;

    /**
     * @brief 获取电机减速比配置
     */
    const std::vector<double>& getMotorGearRatio() const;

    /**
     * @brief 获取底层 Pinocchio Model 的 const 引用（线程安全只读访问）。
     *
     * 用途：外部需要直接调用 pinocchio::computeAllTerms 等算法时，
     * 可通过此接口获取 model，配合自己的 pinocchio::Data 使用。
     */
    const pinocchio::Model& getPinocchioModel() const;

    /**
     * @brief 计算重力补偿力矩
     * 
     * @param q 关节位置
     * @return 重力力矩向量
     */
    Eigen::VectorXd computeGravityTorque(const Eigen::VectorXd& q);

    /**
     * @brief 计算逆动力学（RNEA）
     * 
     * 计算 τ = M(q)a + C(q,v)v + g(q)
     * 
     * @param q 关节位置
     * @param v 关节速度
     * @param a 期望关节加速度
     * @return 所需关节力矩
     */
    Eigen::VectorXd computeInverseDynamics(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a);


    /**
     * @brief 计算正动力学（ABA）
     * 
     * 计算 a = M(q)^(-1) * (τ - C(q,v)v - g(q))
     * 
     * @param q 关节位置
     * @param v 关节速度
     * @param tau 期望关节力矩
     * @return 关节加速度
     */
    Eigen::VectorXd computeForwardDynamics(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& tau);

    /**
     * @brief 计算惯性矩阵 M(q)（仅连杆惯量，用于仿真）
     * 
     * 自动选择计算方法：优先使用 CRBA，失败时回退到数值方法
     * 
     * @param q 关节位置
     * @return 惯性矩阵
     */
    Eigen::MatrixXd computeInertiaMatrix(const Eigen::VectorXd& q);
    
    /**
     * @brief 使用 CRBA 算法计算惯性矩阵 M(q)
     * 
     * Composite Rigid Body Algorithm，计算效率高（O(n²)），
     * 但在某些关节配置下可能因数值问题失败
     * 
     * @param q 关节位置
     * @return 惯性矩阵（仅连杆惯量）
     * @throws std::runtime_error 当 CRBA 遇到数值问题时抛出异常
     */
    Eigen::MatrixXd computeInertiaMatrixCRBA(const Eigen::VectorXd& q);
    
    /**
     * @brief 使用数值方法计算惯性矩阵 M(q)（基于逆动力学）
     * 
     * 该方法通过逆动力学算法（RNEA）计算惯性矩阵，避免 CRBA 的数值问题。
     * 原理：tau = M(q)*ddq + C(q,dq)*dq + g(q)
     *       当 dq=0 时，C(q,dq)=0
     *       对每个关节 i，设置 ddq=[0,...,1,...,0]（第i个为1）
     *       则 M(:,i) = tau(q, 0, ddq) - g(q)
     * 
     * @param q 关节位置
     * @return 惯性矩阵（仅连杆惯量）
     */
    Eigen::MatrixXd computeInertiaMatrixNumerical(const Eigen::VectorXd& q);
    
    /**
     * @brief 计算惯性矩阵 M(q)（连杆惯量 + 电机转子惯量，用于真实机器人）
     * 
     * @param q 关节位置
     * @return 惯性矩阵（包含电机转子等效惯量）
     */
    Eigen::MatrixXd computeInertiaMatrixWithMotor(const Eigen::VectorXd& q);

    /**
     * @brief 计算科氏力和离心力项 C(q,v)v
     * 
     * @param q 关节位置
     * @param v 关节速度
     * @return 非线性力矩
     */
    Eigen::VectorXd computeNonlinearEffects(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v);

    /**
     * @brief 计算末端执行器雅可比矩阵
     * 
     * @param q 关节位置
     * @return 6xN 雅可比矩阵（线速度 + 角速度）
     */
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q);

    /**
     * @brief 计算末端执行器雅可比矩阵（线程安全版本，使用外部 Data 对象）
     * 
     * @param q 关节位置
     * @param external_data 外部提供的 pinocchio::Data 对象（通过 createData() 创建）
     * @return 6xN 雅可比矩阵（线速度 + 角速度）
     */
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q, pinocchio::Data& external_data) const;

    /**
     * @brief 计算非线性效应 h(q,v)=C(q,v)v+g(q)（线程安全版本）
     *
     * 供在独立线程中运行的力估计器使用，避免与控制线程共享 impl_->data_。
     *
     * @param q            关节位置
     * @param v            关节速度
     * @param external_data 外部提供的 pinocchio::Data（通过 createData() 创建）
     * @return model_dof 维非线性力矩向量
     */
    Eigen::VectorXd computeNonlinearEffects(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        pinocchio::Data& external_data) const;

    /**
     * @brief 使用 CRBA 计算惯性矩阵（线程安全版本）
     *
     * @param q            关节位置
     * @param external_data 外部提供的 pinocchio::Data
     * @return model_dof × model_dof 惯性矩阵
     */
    Eigen::MatrixXd computeInertiaMatrixCRBA(
        const Eigen::VectorXd& q,
        pinocchio::Data& external_data) const;

    /**
     * @brief 计算含电机惯量的惯性矩阵（线程安全版本）
     *
     * @param q            关节位置
     * @param external_data 外部提供的 pinocchio::Data
     * @return model_dof × model_dof 惯性矩阵（含电机转子折合惯量）
     */
    Eigen::MatrixXd computeInertiaMatrixWithMotor(
        const Eigen::VectorXd& q,
        pinocchio::Data& external_data) const;

    /**
     * @brief 逆动力学 RNEA（线程安全版本）：τ = M(q)a + C(q,v)v + g(q)
     *
     * @param q            关节位置
     * @param v            关节速度
     * @param a            关节加速度
     * @param external_data 外部提供的 pinocchio::Data
     * @return model_dof 维力矩向量
     */
    Eigen::VectorXd computeInverseDynamics(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a,
        pinocchio::Data& external_data) const;

    /**
     * @brief 计算 Coriolis 矩阵 C(q,v)（线程安全版本）
     *
     * Pinocchio 使用 Christoffel 符号定义 C，满足 (Ṁ − 2C) 为反对称矩阵，
     * 等价于 Ṁ = C + C^T。
     *
     * 用途：De Luca (2005) 动量观测器正确偏置项为 C^T(q,v)·v − g(q)，
     * 而非错误的 −(C(q,v)·v + g(q))。
     *
     * @param q            关节位置
     * @param v            关节速度
     * @param external_data 外部提供的 pinocchio::Data（线程安全）
     * @return model_dof × model_dof 的 Coriolis 矩阵 C(q,v)
     */
    Eigen::MatrixXd computeCoriolisMatrix(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        pinocchio::Data& external_data) const;

    /**
     * @brief 计算重力力矩 g(q)（线程安全版本）
     *
     * 等价于 computeNonlinearEffects(q, 0, data)，但语义更清晰。
     *
     * @param q            关节位置
     * @param external_data 外部提供的 pinocchio::Data
     * @return model_dof 维重力力矩向量
     */
    Eigen::VectorXd computeGravityTorque(
        const Eigen::VectorXd& q,
        pinocchio::Data& external_data) const;

    /**
     * @brief 计算正运动学（末端位姿）
     * 
     * @param q 关节位置
     * @return 4x4 齐次变换矩阵
     */
    Eigen::Matrix4d computeForwardKinematics(const Eigen::VectorXd& q);

    /**
     * @brief 计算正运动学（线程安全版本，使用外部 Data 对象）
     * 
     * 该重载接受一个外部 pinocchio::Data 对象，用于在多线程场景下
     * 避免并发修改内部 data_ 导致的数据竞争。
     * 
     * @param q 关节位置
     * @param external_data 外部提供的 pinocchio::Data 对象（通过 createData() 创建）
     * @return 4x4 齐次变换矩阵
     */
    Eigen::Matrix4d computeForwardKinematics(const Eigen::VectorXd& q, pinocchio::Data& external_data) const;

    /**
     * @brief 与数值 IK 一致的加权末端误差范数：‖ W · log6(T_des.actInv(T_ee(q))) ‖₂（LOCAL/框架与 IK 相同）
     * @param q_joint 长度 nv 或 nq，与 solveInverseKinematicsIterator 的 init_pos 一致
     */
    double computeWeightedIkPoseErrorNorm(
        const Eigen::VectorXd& q_joint,
        const Eigen::Matrix4d& T_des,
        const Eigen::Matrix<double, 6, 1>& weight_ik);

    /**
     * @brief 创建一个新的 pinocchio::Data 对象
     * 
     * 用于多线程场景下，每个线程使用独立的 Data 对象，
     * 避免并发修改同一个 data_ 导致崩溃。
     * 
     * @return pinocchio::Data 新的 Data 对象
     */
    pinocchio::Data createData() const;

    /**
     * @brief 计算末端执行器速度（线速度 + 角速度）
     *
     * @param q 关节位置
     * @param q_dot 关节速度
     * @return 6维速度向量 [vx, vy, vz, wx, wy, wz]
     */
    Eigen::VectorXd computeEndEffectorVelocity(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& q_dot);

    /**
     * @brief 计算末端执行器速度（线程安全版本，使用外部 Data 对象）
     *
     * @param q 关节位置
     * @param q_dot 关节速度
     * @param external_data 外部提供的 pinocchio::Data 对象（通过 createData() 创建）
     * @return 6维速度向量 [vx, vy, vz, wx, wy, wz]
     */
    Eigen::VectorXd computeEndEffectorVelocity(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& q_dot,
        pinocchio::Data& external_data) const;

    /**
     * @brief 设置逆运动学求解器配置
     * 
     * @param config IK配置参数
     */
    void setIteratorConfig(const IKConfig& config);

    /**
     * @brief 使用IK方法求解逆运动学
     * 
     * @param init_pos 初始关节位置（如果为空则使用零位置）
     * @param target_pose 目标位姿（4x4齐次变换矩阵）
     * @param joint_positions 输出：求解得到的关节位置
     * @param q_ref 参考关节位置（用于零空间控制，可选）
     * @return true 求解成功
     * @return false 求解失败
     */
    bool solveInverseKinematicsIterator(
        const Eigen::VectorXd& init_pos,
        const Eigen::Matrix4d& target_pose,
        Eigen::VectorXd& joint_positions,
        const Eigen::VectorXd& q_ref = Eigen::VectorXd(),
        const Eigen::VectorXd* ik_joint_min = nullptr,
        const Eigen::VectorXd* ik_joint_max = nullptr,
        double ik_joint_clamp_margin = -1.0);

    /**
     * @brief AR5 半解析+φ/ψ 扫描，生成数值 IK 初值（需 cfg.enable；失败返回 false）
     * @param q_hint_full / q_joint_ref_full 通常同为上一周期数值 IK 解（首拍可无历史，用当前实测）
     */
    bool tryComputeAr5AnalyticalIkSeed(
        const Ar5AnalyticalIkControlConfig& cfg,
        Ar5AnalyticalIkSeedRuntimeState& state_io,
        const Eigen::VectorXd& q_hint_full,
        const Eigen::VectorXd& q_joint_ref_full,
        const Eigen::Matrix4d& T_des,
        const Eigen::Matrix<double, 6, 1>& weight_ik,
        Eigen::VectorXd& q_seed_out);

    /**
     * @brief 获取关节位置限位
     * 
     * @return JointLimits 包含 min_position 和 max_position 的结构体
     */
    JointLimits getJointLimits() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    void extractJointLimitsFromModel();
    Eigen::VectorXd clampJointPositions(const Eigen::VectorXd& q) const;
};

}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__ROBOT_MODEL_HPP_


