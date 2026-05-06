#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>
#include <memory>

namespace teleop_app {
namespace solvers {

/**
 * @brief 夹爪类型
 */
enum class GripperType {
    PARALLEL,      // 平行夹爪（两指相对运动，输出距离）
    ANGULAR,       // 角度夹爪（单指转动，输出角度）
    MULTI_FINGER   // 多指夹爪（多个关节，输出多个值）
};

/**
 * @brief 夹爪解算结果
 */
struct GripperResult {
    GripperType type;              // 夹爪类型
    double value;                  // 主要值（距离或角度）
    std::vector<double> values;    // 多指夹爪的所有关节值
    bool valid;                    // 是否有效
    
    GripperResult() : type(GripperType::PARALLEL), value(0.0), valid(false) {}
};

/**
 * @brief 夹爪解算器参数
 */
struct GripperSolverParams {
    GripperType type;              // 夹爪类型
    int joint_count;               // 关节数量
    
    // 平行夹爪参数
    double finger_length;          // 指长（用于距离计算）
    double max_distance;           // 最大开口距离
    double min_distance;           // 最小开口距离
    
    // 角度夹爪参数
    double max_angle;              // 最大角度（弧度）
    double min_angle;              // 最小角度（弧度）
    
    // 关节映射关系
    std::vector<double> joint_scales;  // 关节比例系数
    std::vector<double> joint_offsets; // 关节偏移量
    // 机器人类型标识（例如 "imeta"），供解算器选择厂商特定映射
    std::string robot_type;
    
    GripperSolverParams() 
        : type(GripperType::PARALLEL)
        , joint_count(1)
        , finger_length(0.1)
        , max_distance(0.1)
        , min_distance(0.0)
        , max_angle(1.57)
        , min_angle(0.0) {}
};

/**
 * @brief 夹爪解算器状态
 */
enum class GripperSolverStatus {
    UNINITIALIZED,  // 未初始化
    INITIALIZED,    // 已初始化
    ERROR          // 错误状态
};

/**
 * @brief 夹爪解算器类（独立算法组件）
 * 
 * 设计理念：
 * - 纯算法组件，不包含ROS相关功能
 * - 使用Pimpl模式隐藏实现细节
 * - 将关节状态转换为夹爪开合距离或角度
 * - 支持多种夹爪类型
 */
class GripperSolver {
public:
    GripperSolver();
    ~GripperSolver();

    /**
     * @brief 初始化解算器
     * @param params 解算器参数
     * @return true 初始化成功
     */
    bool initialize(const GripperSolverParams& params);

    /**
     * @brief 计算夹爪状态（从关节位置计算夹爪开合程度）
     * @param joint_positions 关节位置向量
     * @return 夹爪状态结果
     */
    GripperResult compute(const Eigen::VectorXd& joint_positions);

    /**
     * @brief 反向计算（从期望的夹爪开合程度计算关节位置）
     * @param desired_value 期望的距离或角度
     * @return 关节位置向量
     */
    Eigen::VectorXd computeInverse(double desired_value);

    /**
     * @brief 获取关节数量
     */
    int getJointCount() const;

    /**
     * @brief 获取夹爪类型
     */
    GripperType getGripperType() const;

    /**
     * @brief 获取解算器状态
     */
    GripperSolverStatus getStatus() const;

private:
    // Pimpl模式：隐藏实现细节
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace solvers
} // namespace teleop_app
