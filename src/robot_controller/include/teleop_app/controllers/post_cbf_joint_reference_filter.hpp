#pragma once

#include <algorithm>
#include <cmath>
#include <deque>

#include <Eigen/Core>

#include "teleop_app/controllers/data_types.hpp"

namespace teleop_app {
namespace controllers {

/**
 * 仅对下发 MIT 的**臂关节** cmd.position/cmd.velocity 做平滑；
 * 不改变 CBF 内部的 q_target、v_target。
 *
 * - enable：q_out ← 对 q_cbf 先做滑动算术均值，再一阶低通 q_f←(1−a)·q_f+a·q_ma。
 *           v_out ← 对 q_f 差分后再一阶低通。
 * - !enable：q_out ← q_cbf，v_out ← v_cbf。
 */
class PostCbfJointReferenceFilter {
public:
    void reset() {
        initialized_ = false;
        q_filt_.resize(0);
        v_filt_.resize(0);
        q_cbf_ma_window_.clear();
        last_ma_n_ = 0;
        last_ma_win_ = 0;
    }

    /**
     * @param q_cbf_arm  CBF 链得到的关节目标位置（仅臂，长度 n）
     * @param q_out_arm  输出：写入 cmd.position 的臂关节分量
     * @param v_out_arm  输出：写入 cmd.velocity 的臂关节分量
     */
    void filterMitArmReference(const PostCbfJointReferenceFilterConfig& cfg,
        const Eigen::VectorXd& q_cbf_arm,
        const Eigen::VectorXd& v_cbf_arm,
        double dt,
        Eigen::VectorXd& q_out_arm,
        Eigen::VectorXd& v_out_arm) {
        const int n = static_cast<int>(q_cbf_arm.size());
        if (n <= 0) {
            return;
        }
        q_out_arm.resize(n);
        v_out_arm.resize(n);

        if (!cfg.enable) {
            q_out_arm = q_cbf_arm;
            if (v_cbf_arm.size() == n && v_cbf_arm.allFinite()) {
                v_out_arm = v_cbf_arm;
            } else {
                v_out_arm.setZero();
            }
            initialized_ = false;
            q_filt_.resize(0);
            v_filt_.resize(0);
            q_cbf_ma_window_.clear();
            last_ma_n_ = 0;
            last_ma_win_ = 0;
            return;
        }

        if (!q_cbf_arm.allFinite()) {
            q_out_arm = q_cbf_arm;
            v_out_arm.setZero();
            return;
        }

        const double a = std::clamp(cfg.position_new_weight, 1e-6, 1.0);
        const double av = std::clamp(cfg.velocity_new_weight, 1e-6, 1.0);
        const int win = std::clamp(cfg.position_mean_window, 1, 64);
        if (n != last_ma_n_ || win != last_ma_win_) {
            q_cbf_ma_window_.clear();
            last_ma_n_ = n;
            last_ma_win_ = win;
        }

        q_cbf_ma_window_.push_back(q_cbf_arm);
        while (static_cast<int>(q_cbf_ma_window_.size()) > win) {
            q_cbf_ma_window_.pop_front();
        }

        Eigen::VectorXd q_ma = Eigen::VectorXd::Zero(n);
        for (const Eigen::VectorXd& s : q_cbf_ma_window_) {
            q_ma += s;
        }
        q_ma /= static_cast<double>(q_cbf_ma_window_.size());

        if (!initialized_ || q_filt_.size() != n) {
            q_filt_ = q_ma;
            v_filt_ = Eigen::VectorXd::Zero(n);
            initialized_ = true;
            q_out_arm = q_filt_;
            v_out_arm = v_filt_;
            return;
        }

        const Eigen::VectorXd q_prev = q_filt_;
        q_filt_ = (1.0 - a) * q_filt_ + a * q_ma;
        Eigen::VectorXd v_raw = Eigen::VectorXd::Zero(n);
        if (dt > 1e-9) {
            v_raw = (q_filt_ - q_prev) / dt;
        }
        if (v_filt_.size() != n) {
            v_filt_ = v_raw;
        } else {
            v_filt_ = (1.0 - av) * v_filt_ + av * v_raw;
        }
        q_out_arm = q_filt_;
        v_out_arm = v_filt_;
    }

private:
    bool initialized_{false};
    Eigen::VectorXd q_filt_;
    Eigen::VectorXd v_filt_;
    std::deque<Eigen::VectorXd> q_cbf_ma_window_;
    int last_ma_n_{0};
    int last_ma_win_{0};
};

}  // namespace controllers
}  // namespace teleop_app
