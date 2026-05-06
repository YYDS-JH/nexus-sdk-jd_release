#pragma once
#include <Eigen/Core>
#include <memory>

class SCurveProfile {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SCurveProfile(int dof);
    ~SCurveProfile();
    SCurveProfile(SCurveProfile&&) noexcept;
    SCurveProfile& operator=(SCurveProfile&&) noexcept;

    void SetMaxVel(const Eigen::VectorXd& max_vel);
    void SetMaxAcc(const Eigen::VectorXd& max_acc);
    void SetMaxJerk(const Eigen::VectorXd& max_jerk);
    void SetStartPosition(const Eigen::VectorXd& start);
    void SetTargetPosition(const Eigen::VectorXd& target);
    void SetVelScale(double scale);

    void Reset();
    void Update(double dt);
    bool IsFinished() const;

    const Eigen::VectorXd& GetPosition() const;
    const Eigen::VectorXd& GetVelocity() const;
    const Eigen::VectorXd& GetAcceleration() const;
    double GetSyncTime() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    static void ComputeProfile(Impl& impl);
};
