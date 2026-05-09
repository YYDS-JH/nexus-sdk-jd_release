/**
 * @file ar5_dynamics_force_estimator.hpp
 * @brief AR5 七轴臂动力学外力估计器 —— v6。
 *
 * 参考：mc_rtc / mc_residual_estimation (CNRS-UM LIRMM)
 *
 * 四种方法：quasi_static / momentum_observer / inv_dyn_residual / delta_momentum
 * 线程安全：独立 pinocchio::Data，不与控制线程共享。
 */

#pragma once

#include "teleop_app/controllers/data_types.hpp"
#include "teleop_app/controllers/robot_model.hpp"

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace teleop_app {
namespace controllers {

class Ar5DynamicsForceEstimator {
public:
    Ar5DynamicsForceEstimator();
    ~Ar5DynamicsForceEstimator();

    bool configure(const Ar5DynamicsForceEstimatorConfig& cfg,
                   RobotModel* model, int arm_dof);

    CartesianForce estimate(const Eigen::VectorXd& q_model,
                            const Eigen::VectorXd& v_model,
                            const Eigen::VectorXd& tau_arm);

    void reset();
    bool isConfigured() const { return configured_; }

    const Eigen::VectorXd& debugTauMeas()  const { return debug_tau_meas_; }
    const Eigen::VectorXd& debugTauModel() const { return debug_tau_model_; }
    const Eigen::VectorXd& debugTauExt()   const { return debug_tau_ext_; }
    const Eigen::VectorXd& debugTauComp()  const { return debug_tau_comp_; }

private:
    enum class Method { QUASI_STATIC, MOMENTUM_OBSERVER, INV_DYN_RESIDUAL, DELTA_MOMENTUM };

    Ar5DynamicsForceEstimatorConfig cfg_;
    Method method_{Method::MOMENTUM_OBSERVER};
    RobotModel* robot_model_{nullptr};
    int arm_dof_{7};
    int model_dof_{7};
    int nv_{7};
    bool configured_{false};

    std::unique_ptr<pinocchio::Data> dyn_data_;
    std::unique_ptr<pinocchio::Data> dbg_data_;  // debug RNEA 专用，不与算法共享
    Eigen::VectorXd armature_;

    // 动量观测器
    Eigen::VectorXd sigma_;
    Eigen::VectorXd p_zero_;
    Eigen::VectorXd r_prev_;
    bool momentum_initialized_{false};

    // 逆动力学残差
    Eigen::VectorXd v_filtered_;
    Eigen::VectorXd v_model_prev_;
    Eigen::VectorXd a_filtered_;
    bool inv_dyn_initialized_{false};

    // 差分动量观测器
    Eigen::VectorXd dm_p_prev_;
    Eigen::VectorXd dm_v_prev_;
    Eigen::MatrixXd dm_M_prev_;
    Eigen::VectorXd dm_r_filtered_;
    bool dm_initialized_{false};

    // 输出滤波
    Eigen::Matrix<double, 6, 1> F_filtered_;
    bool filter_initialized_{false};

    // dt 测量
    std::chrono::steady_clock::time_point last_call_time_;
    bool timing_initialized_{false};

    // debug 数据（每次 estimate() 后更新）
    Eigen::VectorXd debug_tau_meas_;
    Eigen::VectorXd debug_tau_model_;
    Eigen::VectorXd debug_tau_ext_;
    Eigen::VectorXd debug_tau_comp_;
    Eigen::VectorXd debug_v_prev_;
    Eigen::VectorXd debug_a_filt_;  // 滤波后的加速度
    bool debug_initialized_{false};


    // ── 残差补偿 ─────────────────────────────────────────────
    static constexpr int kResidualFeatureDim = 106; // +49 v_i*sin(q_j)
    Eigen::MatrixXd compensation_coeffs_;            // (nv_ × kResidualFeatureDim)
    bool compensation_loaded_{false};

    Eigen::VectorXd buildFeatureVector(const Eigen::VectorXd& q,
                                       const Eigen::VectorXd& v);
    Eigen::VectorXd predictResidual(const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& v);
    bool loadCompensationModel(const std::string& path);

    Eigen::VectorXd estimateQuasiStatic(const Eigen::VectorXd& tau);
    Eigen::VectorXd estimateMomentumObserver(const Eigen::VectorXd& q,
                                              const Eigen::VectorXd& v,
                                              const Eigen::VectorXd& tau,
                                              double dt);
    Eigen::VectorXd estimateInvDynResidual(const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& v,
                                            const Eigen::VectorXd& tau,
                                            double dt);
    Eigen::VectorXd estimateDeltaMomentum(const Eigen::VectorXd& q,
                                           const Eigen::VectorXd& v,
                                           const Eigen::VectorXd& tau,
                                           double dt);
    CartesianForce mapToCartesian(const Eigen::VectorXd& tau_ext,
                                   const Eigen::VectorXd& q,
                                   double dt);
};

}  // namespace controllers
}  // namespace teleop_app
