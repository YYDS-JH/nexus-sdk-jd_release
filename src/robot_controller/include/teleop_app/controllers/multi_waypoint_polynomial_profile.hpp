#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

/**
 * @brief 多 waypoint 五次多项式轨迹插补器。
 *
 * 输入整条关节空间路径 [q0, q1, ..., qN]，中间 waypoint 自动估计非零速度，
 * 只在终点停车。每个控制周期 Update(dt) 后输出当前 q/qd/qdd。
 */
class MultiWaypointPolynomialProfile {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit MultiWaypointPolynomialProfile(int dof);
    ~MultiWaypointPolynomialProfile();
    MultiWaypointPolynomialProfile(MultiWaypointPolynomialProfile&&) noexcept;
    MultiWaypointPolynomialProfile& operator=(MultiWaypointPolynomialProfile&&) noexcept;

    bool SetTrajectory(
        const std::vector<Eigen::VectorXd>& waypoints,
        const std::vector<double>& segment_durations,
        const Eigen::VectorXd& start_velocity,
        double waypoint_velocity_scale);

    void Reset();
    void Update(double dt);
    bool IsFinished() const;

    const Eigen::VectorXd& GetPosition() const;
    const Eigen::VectorXd& GetVelocity() const;
    const Eigen::VectorXd& GetAcceleration() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    static void ComputeCoefficients(Impl& impl);
};
