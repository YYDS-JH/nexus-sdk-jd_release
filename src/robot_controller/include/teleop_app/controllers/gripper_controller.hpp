#ifndef TELEOP_APP__CONTROLLERS__GRIPPER_CONTROLLER_HPP_
#define TELEOP_APP__CONTROLLERS__GRIPPER_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <array>
#include "teleop_app/controllers/data_types.hpp"

namespace teleop_app {
namespace controllers {

/**
 * @brief 夹爪控制器类
 * 
 * 功能：
 * 1. 管理夹爪关节状态
 * 2. 查表转换（电机角度(rad) <-> 开合距离(mm)，输出URDF夹爪关节位移(m)）
 * 4. 计算两夹爪位移量
 * 5. 主从遥操作时的夹爪指令转换
 */
class GripperController {
public:
    enum class GripperForceFeedbackState {
        FREE = 0,
        HOLDING = 1
    };

    GripperController();
    ~GripperController();
    GripperController(GripperController&&) noexcept;
    GripperController& operator=(GripperController&&) noexcept;

    // ===== 五次多项式拟合（启动时只计算一次）=====
    // 使用 x_norm = (x - x_mean) / x_scale 做归一化，提高数值稳定性。
    // 该类型仅用于实现细节：从表数据拟合 poly5，运行时优先用 poly5，失败则回退线性插值（保底）。
    struct Poly5Fit {
        bool valid{false};
        double x_mean{0.0};
        double x_scale{1.0};
        std::array<double, 6> c{};  // x_norm 的五次多项式系数（从高次到常数项）
        double rmse{0.0};
        double max_abs_err{0.0};
    };

    /**
     * @brief 初始化控制器
     * @param config 控制器配置
     * @return true 初始化成功
     */
    bool initialize(const GripperConfig& config);

    /**
     * @brief 更新夹爪状态（从关节状态计算所有相关量）
     * @param gripper_pos 夹爪关节位置（1维量，通常是第一个关节的位置）
     * @param gripper_vel 夹爪关节速度（1维量，通常是第一个关节的速度）
     * @param timestamp 时间戳
     * @return GripperState 更新后的夹爪状态
     */
    GripperState updateState(
        double gripper_pos,
        double gripper_vel,
        double timestamp = 0.0
    );

    /**
     * @brief 获取当前夹爪状态
     */
    const GripperState& getCurrentState() const;

    /**
     * @brief 获取配置
     */
    const GripperConfig& getConfig() const;

    /**
     * @brief 获取本机夹爪角色（主臂/从臂），用于按角色判断是否使用接收到的 gripper 力反馈等
     */
    ArmRole getSelfRole() const;

    /**
     * @brief 检查是否已初始化
     */
    bool isInitialized() const;

    /**
     * @brief 检查是否为 AR5 吸盘型夹爪
     * @return true 表示直接将收到的 gripper 原始值作为 pos 透传，不做任何转换
     */
    bool isAr5SuctionCup() const;

    /**
     * @brief 检查是否为 AR5 夹爪（gripper 或 suction_cup）
     * @return true 表示直接将收到的 gripper 原始值作为 pos 透传，不做任何转换
     */
    bool isAr5Gripper() const;

    /**
     * @brief 查表：从旋转角度查找归一化开合度（真实机器人）
     * @param motor_angle 旋转电机角度 (rad)
     * @param role 主臂或从臂标识
     * @return 归一化开合度 [0, 1]，0表示最小开合，1表示最大开合
     */
    double strokeToOpenning(double motor_angle, ArmRole role) const;
    
    /**
     * @brief 查表：从归一化开合度查找旋转角度（真实机器人）
     * @param normalized_opening 归一化开合度 [0, 1]，0表示最小开合，1表示最大开合
     * @param role 主臂或从臂标识
     * @return 旋转电机角度 (rad)
     */
    double openningToStroke(double normalized_opening, ArmRole role) const;

    /**
     * @brief 估计夹爪力（用于主从遥操作力反馈）
     *
     * 基于状态机（FREE/HOLDING）估计夹爪力反馈：
     *   - FREE: F = 0
     *   - HOLDING: F = lower + (upper - lower) * tanh(t / tau)，t 为驻留时间
     *
     * @param master_opening_normalized 主臂夹爪归一化开合度 [0, 1]
     * @param slave_opening_normalized 从臂夹爪归一化开合度 [0, 1]
     * @param slave_gripper_velocity 从臂夹爪关节速度 (rad/s)
     * @param timestamp 当前时刻（秒）
     * @return 夹爪力 (N)
     */
    double estimateGripperForce(double master_opening_normalized,
                                double slave_opening_normalized,
                                double slave_gripper_velocity,
                                double timestamp);

    /**
     * @brief 重置夹爪力反馈状态机内部状态
     */
    void resetForceFeedbackState();

    /**
     * @brief 获取夹爪关节角度范围
     * @param role 主臂或从臂标识
     * @param min_angle 输出最小角度 (rad)
     * @param max_angle 输出最大角度 (rad)
     * @return 是否成功获取（表已加载）
     */
    bool getAngleRange(ArmRole role, double& min_angle, double& max_angle) const;

    /**
     * @brief 将开合距离归一化到 [0, 1]
     * @param opening_distance_mm 开合距离 (mm)
     * @param role 主臂或从臂标识，用于选择对应的开合度范围
     * @return 归一化值 [0, 1]
     */
    double normalizeOpening(double opening_distance_mm, ArmRole role) const;
    
    /**
     * @brief 将归一化值反归一化为开合距离
     * @param normalized_opening 归一化值 [0, 1]
     * @param role 主臂或从臂标识，用于选择对应的开合度范围
     * @return 开合距离 (mm)
     */
    double denormalizeOpening(double normalized_opening, ArmRole role) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    static ArmRole parseSelfRoleOrDefault(const Impl& impl);
    static bool loadStrokeTable(Impl& impl, const std::string& table_path, bool is_master);
};

}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__GRIPPER_CONTROLLER_HPP_

