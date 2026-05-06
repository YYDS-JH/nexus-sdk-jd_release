#pragma once
#include <Eigen/Core>
#include <memory>

/**
 * @brief 多项式插值轨迹规划器（五次多项式）
 * 
 * 使用五次多项式插值，满足位置、速度、加速度的边界条件：
 * - 起始位置、速度、加速度
 * - 目标位置、速度、加速度（通常速度和加速度为0）
 */
class PolynomialProfile {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PolynomialProfile(int dof);
    ~PolynomialProfile();
    PolynomialProfile(PolynomialProfile&&) noexcept;
    PolynomialProfile& operator=(PolynomialProfile&&) noexcept;

    /**
     * @brief 设置起始位置
     */
    void SetStartPosition(const Eigen::VectorXd& start);

    /**
     * @brief 设置目标位置
     */
    void SetTargetPosition(const Eigen::VectorXd& target);

    /**
     * @brief 设置起始速度（可选，默认为0）
     */
    void SetStartVelocity(const Eigen::VectorXd& start_vel);

    /**
     * @brief 设置目标速度（可选，默认为0）
     */
    void SetTargetVelocity(const Eigen::VectorXd& target_vel);

    /**
     * @brief 设置起始加速度（可选，默认为0）
     */
    void SetStartAcceleration(const Eigen::VectorXd& start_acc);

    /**
     * @brief 设置目标加速度（可选，默认为0）
     */
    void SetTargetAcceleration(const Eigen::VectorXd& target_acc);

    /**
     * @brief 设置轨迹总时间
     */
    void SetDuration(double duration);

    /**
     * @brief 重置规划器（时间归零，位置设为起始位置）
     */
    void Reset();

    /**
     * @brief 更新规划器（时间步进）
     * @param dt 时间步长
     */
    void Update(double dt);

    /**
     * @brief 检查是否完成
     */
    bool IsFinished() const;

    /**
     * @brief 获取当前位置
     */
    const Eigen::VectorXd& GetPosition() const;

    /**
     * @brief 获取当前速度
     */
    const Eigen::VectorXd& GetVelocity() const;

    /**
     * @brief 获取当前加速度
     */
    const Eigen::VectorXd& GetAcceleration() const;

    /**
     * @brief 获取轨迹总时间
     */
    double GetDuration() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    static void ComputeCoefficients(Impl& impl);
};

