#ifndef TELEOP_APP__CONTROLLERS__PID_CONTROLLER_HPP_
#define TELEOP_APP__CONTROLLERS__PID_CONTROLLER_HPP_

#include <Eigen/Core>
#include <memory>

namespace teleop_app {
namespace controllers {

/**
 * @brief PID控制器类（带积分抗饱和）
 * 
 * 实现标准PID控制算法：
 * u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
 * 
 * 特性：
 * - 支持多维度（每个关节独立）
 * - 积分抗饱和（anti-windup）
 * - 积分项限幅
 */
class PIDController {
public:
    /**
     * @brief 构造函数
     * 
     * @param dof 自由度数量
     */
    explicit PIDController(int dof);
    
    /**
     * @brief 析构函数
     */
    ~PIDController();

    PIDController(PIDController&&) noexcept;
    PIDController& operator=(PIDController&&) noexcept;
    
    /**
     * @brief 设置PID增益
     * 
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     */
    void setGains(const Eigen::VectorXd& kp, 
                  const Eigen::VectorXd& ki, 
                  const Eigen::VectorXd& kd);
    
    /**
     * @brief 设置积分力矩限幅
     * 
     * @param i_max 积分力矩最大值（绝对值，N·m）
     */
    void setIntegralLimit(const Eigen::VectorXd& i_max);
    
    /**
     * @brief 设置总输出限幅（用于积分抗饱和）
     * 
     * @param output_max 总输出最大值（绝对值，N·m）
     */
    void setOutputLimit(const Eigen::VectorXd& output_max);
    
    /**
     * @brief 计算PID控制输出
     * 
     * @param error 当前误差
     * @param dt 时间步长（秒）
     * @return PID控制输出
     */
    Eigen::VectorXd compute(const Eigen::VectorXd& error, double dt);
    
    /**
     * @brief 获取积分力矩项（用于前馈）
     * 
     * @return 积分力矩（N·m）
     */
    Eigen::VectorXd getIntegralTerm() const;
    
    /**
     * @brief 重置积分项
     */
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace controllers
}  // namespace teleop_app

#endif  // TELEOP_APP__CONTROLLERS__PID_CONTROLLER_HPP_

