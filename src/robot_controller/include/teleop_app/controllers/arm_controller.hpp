#ifndef TELEOP_APP__CONTROLLERS__ARM_CONTROLLER_HPP_
#define TELEOP_APP__CONTROLLERS__ARM_CONTROLLER_HPP_

#include "teleop_app/controllers/data_types.hpp"
#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

namespace teleop_app {
namespace controllers {

class GripperController;

/**
 * @brief 机械臂控制器类（独立算法组件）
 * 
 */
class ArmController {
public:
    ArmController();
    ~ArmController();

    /**
     * @brief 初始化控制器
     * 
     * @param params 控制器参数配置
     * @return true 初始化成功
     * @return false 初始化失败
     * 
     * 功能：
     * - 加载并解析 URDF 运动学模型
     * - 初始化运动学/动力学求解器
     * - 配置控制器参数
     * - 初始化内部状态变量
     */
    bool initialize(const ControllerParams& params);

    /**
     * @brief 计算重力补偿及力反馈控制命令（周期调用）
     * 
     * 使用场景：主臂遥操状态（TELEOP）
     * 
     * 功能：
     * - 提供重力补偿，使用户可以随意拖动主臂
     * - 反馈从臂受到的外力给主臂，实现力觉传递
     * 
     * @param master_joint_state 主臂关节电机状态
     * @param slave_external_force 从臂笛卡尔空间外力
     * @return MitControlCommand 各关节的 MIT 控制指令
     */
    MitControlCommand computeGravityCompensationAndForceFeedback(
        const JointState& master_joint_state,
        const CartesianForce& slave_external_force);

    /**
     * @brief 计算关节空间运动规划控制命令（周期调用）
     * 
     * 使用场景：遥操初始化状态（TELEOP_INIT）
     * 
     * 功能：
     * - 根据 ControllerParams 中的 motion_planning_type 选择规划方法
     * - INTERPOLATE: 使用线性插值规划
     * - SCURVE: 使用S曲线规划
     * 
     * @param current_joint_state 当前主臂关节状态
     * @param target_joint_angles 期望的各关节目标角度
     * @return MotionPlanningResult 包含控制指令和目标到达状态
     */
    MotionPlanningResult computeJointSpaceMotionPlanning(
        const JointState& current_joint_state,
        const std::vector<double>& target_joint_angles);

    /**
     * @brief 使用线性插值计算关节空间运动规划控制命令（周期调用）
     * 
     * 使用场景：遥操初始化状态（TELEOP_INIT）
     * 
     * 功能：
     * - 在关节空间下规划机械臂运动到目标位置
     * - 使用线性插值生成平滑的运动轨迹
     * 
     * @param current_joint_state 当前主臂关节状态
     * @param target_joint_angles 期望的各关节目标角度
     * @return MotionPlanningResult 包含控制指令和目标到达状态
     */
    MotionPlanningResult computeJointSpaceMotionPlanningInterpolate(
        const JointState& current_joint_state,
        const std::vector<double>& target_joint_angles);

    /**
    * @brief 使用S曲线规划计算关节空间运动规划控制命令（周期调用）
    * 
    * 使用场景：遥操初始化状态（TELEOP_INIT）
    * 
    * 功能：
    * - 使用S曲线规划在关节空间下规划机械臂运动到目标位置
    * - 生成平滑的S型运动轨迹
    * - 仅规划机械臂的6个关节，然后与当前夹爪状态拼接
    * - 使用 ControllerParams 中的 ctrl_dt 作为时间步长
    * 
    * @param current_joint_state 当前主臂关节状态
    * @param target_joint_angles 期望的各关节目标角度（arm的6个 + gripper的1个）
    * @return MotionPlanningResult 包含控制指令和目标到达状态
    */
    MotionPlanningResult computeJointMotionPlanningSCurve(
        const JointState& current_joint_state,
        const std::vector<double>& target_joint_angles);

    /**
    * @brief 使用多项式插值规划计算关节空间运动规划控制命令（周期调用）
    * 
    * 使用场景：遥操初始化状态（TELEOP_INIT）
    * 
    * 功能：
    * - 使用五次多项式插值在关节空间下规划机械臂运动到目标位置
    * - 生成平滑的多项式运动轨迹，满足位置、速度、加速度边界条件
    * - 仅规划机械臂的6个关节，然后与当前夹爪状态拼接
    * - 使用 ControllerParams 中的 ctrl_dt 作为时间步长
    * 
    * @param current_joint_state 当前主臂关节状态
    * @param target_joint_angles 期望的各关节目标角度（arm的6个 + gripper的1个）
    * @return MotionPlanningResult 包含控制指令和目标到达状态
    */
    MotionPlanningResult computeJointMotionPlanningPolynomial(
        const JointState& current_joint_state,
        const std::vector<double>& target_joint_angles);
    
    /**
     * @brief 重置关节空间运动规划进度
     * 
     * 作用：在进入复位/初始化状态前调用，使下一次规划从当前实际位置重新开始，
     * 仍然保持“到达后持位”的行为不变。
     */
    void resetMotionPlanning();


    /**
     * @brief 增量模式：弹簧效果控制命令生成（笛卡尔空间定点阻抗控制）
     * 
     * 使用场景：主臂增量遥控模式，定点阻抗弹性模式
     * 
     * 功能：
     * - 根据initial_joint_state_计算参考末端位姿
     * - 实现笛卡尔空间定点阻抗控制
     * - 叠加从臂外力反馈
     * 
     * @param master_joint_state 主臂关节状态
     * @param slave_external_force 从臂笛卡尔空间外力（可选，默认零力）
     * @return MitControlCommand 控制命令
     */
    MitControlCommand computeIncrementalSpringControl(
        const JointState& master_joint_state,
        const CartesianForce& slave_external_force = CartesianForce{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    );

    /**
     * @brief 运动控制器（用于从臂关节阻抗控制）
     * 
     * 使用场景：双边遥操作模式，从臂接收末端位姿指令和夹爪指令
     * 
     * 功能：
     * - 接收期望末端位姿指令
     * - 使用IK解算到关节空间，得到目标关节位置
     * - 接收夹爪开合距离指令，转换为夹爪关节角度
     * - 使用关节阻抗控制器（motion_planning_kp/kd）计算MIT指令
     * - 实现从臂的关节空间阻抗控制
     * 
     * @param current_joint_state 当前关节状态
     * @param desired_pose 期望末端位姿（4x4齐次变换矩阵）
     * @param gripper_target_angle 期望夹爪关节位置（rad），如果为-1.0则保持当前夹爪位置
     * @return MitControlCommand 关节阻抗控制命令
     */
    MitControlCommand computeCartMotionControl(
        const JointState& current_joint_state,
        const Eigen::Matrix4d& desired_pose,
        double gripper_target_angle = -1.0,
        const CartesianForce& slave_external_force = CartesianForce{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    );
    /**
    * @brief 使用 CBF 的笛卡尔空间运动控制（用于从臂）
    * 
    * @param current_joint_state 当前关节状态
    * @param desired_pose 期望末端位姿（4x4齐次变换矩阵）
    * @param gripper_target_angle 期望夹爪关节位置（rad），如果为-1.0则保持当前夹爪位置
    * @param slave_external_force 从臂笛卡尔空间外力
    * @return MitControlCommand 关节阻抗控制命令
    */
    MitControlCommand computeCartMotionControlWithCBF(
        const JointState& current_joint_state,
        const Eigen::Matrix4d& desired_pose,
        double gripper_target_angle = -1.0,
        const CartesianForce& slave_external_force = CartesianForce{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    );

    MitControlCommand computeCartMotionControlWithCBFinSimRobot(
        const JointState& current_joint_state,
        const Eigen::Matrix4d& desired_pose,
        double gripper_target_angle = -1.0,
        const CartesianForce& slave_external_force = CartesianForce{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    );

    /**
     * @brief 冗余臂（如 AR5：7 轴无夹爪）专用：笛卡尔阻抗 + 可选力矩 CBF + 正动力学积分
     *
     * 与 computeCartMotionControlWithCBFinSimRobot 并列；后者保持 6 轴+夹爪扩展语义不变。
     * 调度由节点根据 arm_dof / has_gripper 选择。
     */
    MitControlCommand computeCartMotionControlWithCBFinSimRobotRedundantArm(
        const JointState& current_joint_state,
        const Eigen::Matrix4d& desired_pose,
        double gripper_target_angle = -1.0,
        const CartesianForce& slave_external_force = CartesianForce{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    );

    /**
     * @brief 估计外力（从臂）
     * 
     * 根据配置的估计器类型计算外力：
     * - VIRTUAL_SPRING: F = K * delta_x（根据位置控制偏差和虚拟环境弹性计算碰撞力）
     * - MOMENTUM_BASED: 基于动力学模型和动量的外力观测器（暂未实现）
     * 
     * @param current_joint_state 当前关节状态
     * @param desired_pose 期望末端位姿（4x4齐次变换矩阵）
     * @return CartesianForce 估计的外力
     */
    CartesianForce estimateExternalForce(
        const JointState& current_joint_state,
        const Eigen::Matrix4d& desired_pose
    );

    /**
     * @brief 重置末端力反馈 FSM 状态（DELTA_X_FSM 估计器用），切换估计器类型或重新初始化时调用
     */
    void resetExternalForceFeedbackState();

    /**
     * @brief 计算阻尼控制命令
     * 
     * 使用场景：空闲状态（TELEOP_IDLE）和故障状态（TELEOP_FAULT）
     * 
     * 功能：
     * - 计算低刚度、适度阻尼的控制命令
     * - 使机械臂保持当前位置但允许被动移动
     * 
     * @param current_joint_state 当前关节状态
     * @return MitControlCommand 阻尼控制命令
     */
    MitControlCommand computeDampingCommand(const JointState& current_joint_state);

    /**
     * @brief 获取控制器状态
     * 
     * @return ControllerStatus 控制器当前状态
     */
    ControllerStatus getStatus() const;

    /**
     * @brief 检查控制器是否已初始化
     * 
     * @return true 已初始化
     * @return false 未初始化
     */
    bool isInitialized() const;

    /**
     * @brief 设置初始关节状态
     * 
     * 作用：在进入复位/初始化状态前调用，使下一次规划从初始关节状态重新开始，
     * 仍然保持“到达后持位”的行为不变。
     * 
     * @param initial_joint_state 初始关节状态
     */
    void setInitialJointState(const JointState& initial_joint_state);

    /**
     * @brief 对给定关节状态做 FK（经 expand），得到世界系末端齐次矩阵
     * @return false 模型未加载或关节维数不匹配
     */
    bool tryComputeEndEffectorPoseFromJointState(
        const JointState& joint_state,
        Eigen::Matrix4d& T_world_out) const;

    /**
     * @brief 获取夹爪控制器指针
     * 
     * @return GripperController* 夹爪控制器指针，如果没有夹爪则返回nullptr
     */
    GripperController* getGripperController();

    /**
     * @brief 获取最近一次计算的笛卡尔空间位姿误差
     * 
     * @return const Eigen::VectorXd& 6维笛卡尔空间误差（位置3维 + 姿态3维，世界坐标系）
     */
    const Eigen::VectorXd& getLastCartPoseError() const;

    /**
     * @brief 获取最近一次更新后的末端位姿（6维向量：x, y, z, roll, pitch, yaw）
     */
    const Eigen::VectorXd& getLastPoseEeUpdated() const;

    /**
     * @brief 获取最近一次的期望位姿（6维向量：x, y, z, roll, pitch, yaw）
     */
    const Eigen::VectorXd& getLastDesiredPose() const;

    /**
     * @brief IK（含零空间）得到的关节目标对应的末端 6D：[x,y,z,roll,pitch,yaw]，与 robot_model_ EE 一致
     */
    const Eigen::VectorXd& getLastIkTargetPoseXyzRpy() const;

    /**
     * @brief 冗余臂外环关节位置误差 q_target_extended - q_current（model_dof，rad），与 arm_controller 中 MIT 前计算一致
     */
    const Eigen::VectorXd& getLastJointPosError() const;

    /**
     * @brief 绑定冗余臂调试话题（可选）：T_des 6D；q_des + FK；q_target(CBF 后) + FK；q_meas + 关节跟踪误差；
     * ee_pos_track_err：[e_x,e_y,e_z,‖e‖]，e = p_des(T_des 前三维) − FK(q_meas) 位置（世界系，与 end_effector_frame 一致）
     */
    void setRedundantArmDebugPublishers(
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr T_des_xyz_rpy_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_des_ik_ns_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fk_q_des_xyz_rpy_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_target_after_cbf_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fk_q_target_xyz_rpy_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_meas_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pos_error_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pos_track_err_pub);

    void setForceEstDebugPublishers(
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_meas_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_model_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_ext_pub,
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_comp_pub);

    /**
     * @brief 设置当前控制周期对应的时间（秒），用于静摩擦方波补偿的相位
     * 调用方应在每次控制计算前调用，例如 node_->now().seconds()
     */
    void setCurrentControlTimeSec(double t_sec);

    /**
     * @brief 查询 IK 边界虚拟力是否激活
     */
    bool isIkBoundaryActive() const;

    /**
     * @brief 获取连续 IK 失败次数
     */
    int getConsecutiveIkFailures() const;

    /**
     * @brief 计算 IK 工作空间边界虚拟力
     *
     * 当从臂 IK 连续失败时，以最后有效 IK 解的末端位姿为锚点，
     * 生成虚拟弹簧力将期望位姿拉回可到达区域。
     *
     * @param current_joint_state 当前关节状态
     * @param desired_pose 触发失败的期望位姿 T_des
     * @return 世界系 6D 虚拟边界力
     */
    CartesianForce computeIkBoundaryForce(
        const JointState& current_joint_state,
        const Eigen::Matrix4d& desired_pose) const;

    /**
     * @brief 复位后将 IK 配置同步到运行时 RobotModel（如 vqp_arm_angle_ref_joint_positions）
     */
    void updateRuntimeIkConfig(const IKConfig& ik_config);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    ArmController(const ArmController&) = delete;
    ArmController& operator=(const ArmController&) = delete;
};

}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__ARM_CONTROLLER_HPP_
