#pragma once

#include <array>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "teleop_app/controllers/force_est/ar5_dynamics_force_estimator.hpp"
#include "teleop_app/controllers/data_types.hpp"
#include "teleop_app/controllers/gripper_controller.hpp"
#include "teleop_app/controllers/pid_controller.hpp"
#include "teleop_app/controllers/polynomial_profile.hpp"
#include "teleop_app/controllers/post_cbf_joint_reference_filter.hpp"
#include "teleop_app/controllers/ik/redundant_arm_ik_step.hpp"
#include "teleop_app/controllers/robot_model.hpp"
#include "teleop_app/controllers/s_curve_profile.hpp"
#include "teleop_app/oscbf/utils/collision_pair_manager.hpp"
#include "teleop_app/oscbf/oscbf_controller.hpp"
#include "teleop_app/oscbf/torque_cbf/oscbf_controller.hpp"

#include <pinocchio/multibody/data.hpp>

namespace teleop_app {
namespace controllers {

struct ArmController::Impl {
    static constexpr int EE_FF_DIM = 6;

    ControllerParams params_;
    ControllerStatus status_{ControllerStatus::UNINITIALIZED};

    std::unique_ptr<RobotModel> robot_model_;
    std::unique_ptr<pinocchio::Data> force_est_data_;
    /// 动力学外力估计器（与虚拟弹簧并列，默认未创建 = 虚拟弹簧路径）
    std::unique_ptr<Ar5DynamicsForceEstimator> dynamics_force_estimator_;
    std::unique_ptr<GripperController> gripper_controller_;
    std::unique_ptr<PIDController> joint_pid_controller_;

    Eigen::VectorXd gravity_comp_kp_;
    Eigen::VectorXd gravity_comp_kd_;
    Eigen::VectorXd motion_planning_kp_;
    Eigen::VectorXd motion_planning_kd_;
    Eigen::VectorXd damping_control_kp_;
    Eigen::VectorXd damping_control_kd_;

    JointState initial_joint_state_;

    double planning_percent_{-1.0};
    std::vector<double> planning_start_positions_;
    std::vector<double> last_s_curve_target_;
    std::vector<double> last_polynomial_target_;

    std::unique_ptr<SCurveProfile> s_curve_profile_{nullptr};
    std::unique_ptr<PolynomialProfile> polynomial_profile_{nullptr};

    bool enable_cbf_{false};
    OSCBF::OSCBFController::Config cbf_config_;
    OSCBF::OSCBFController::ObstacleConfig default_obstacle_config_;
    std::shared_ptr<OSCBF::CollisionPairManager> collision_pair_manager_;
    std::unique_ptr<OSCBF::OSCBFController> cbf_controller_;
    std::unique_ptr<RobotModel> robot_model_gripper_locked_;

    bool enable_torque_cbf_{false};
    OSCBF::torque_cbf::OSCBFController::Config torque_cbf_config_;
    OSCBF::torque_cbf::OSCBFController::ObstacleConfig torque_cbf_obstacle_config_;
    std::shared_ptr<OSCBF::torque_cbf::TaskSpaceTorqueConstraint> torque_task_space_constraint_;
    std::shared_ptr<OSCBF::torque_cbf::JointLimitTorqueConstraint> torque_joint_limit_constraint_;
    std::shared_ptr<OSCBF::torque_cbf::JointVelocityTorqueConstraint> torque_joint_velocity_constraint_;
    std::shared_ptr<OSCBF::torque_cbf::SelfCollisionTorqueConstraint> torque_self_collision_constraint_;
    std::unique_ptr<OSCBF::torque_cbf::OSCBFController> torque_cbf_controller_;

    Eigen::Matrix4d pose_target_cbf_{Eigen::Matrix4d::Identity()};
    Eigen::VectorXd q_target;
    Eigen::VectorXd v_target;

    bool pose_target_cbf_initialized_{false};
    bool first_call_cbf_{true};

    double last_filtered_gripper_target_angle_{0.0};
    bool gripper_filter_initialized_{false};
    double gripper_filter_alpha_{0.2};

    RedundantArmIkRuntimeState redundant_arm_ik_runtime_;
    PostCbfJointReferenceFilter post_cbf_joint_ref_filter_;

    // IK 工作空间边界虚拟力
    std::atomic<int> consecutive_ik_failures_{0};
    std::atomic<int> consecutive_ik_successes_{0};
    std::atomic<bool> ik_boundary_active_{false};
    mutable std::mutex ik_boundary_mutex_;
    Eigen::VectorXd last_valid_ik_joint_position_;
    double ik_boundary_activation_time_{-1.0};
    mutable Eigen::Matrix<double, 6, 1> boundary_force_filtered_{Eigen::Matrix<double, 6, 1>::Zero()};
    mutable bool boundary_filter_initialized_{false};

    Eigen::VectorXd friction_coulomb_{Eigen::VectorXd::Zero(0)};
    Eigen::VectorXd filtered_velocity_{Eigen::VectorXd::Zero(0)};
    bool velocity_filter_initialized_{false};
    std::vector<double> velocity_filter_alphas_;
    std::vector<double> friction_velocity_threshold_;
    std::vector<std::deque<int>> velocity_sign_history_;

    double current_control_time_sec_{0.0};

    Eigen::VectorXd last_cart_pose_error_{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd last_pose_ee_updated_{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd last_desired_pose_{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd last_ik_target_pose_xyz_rpy_{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd last_joint_pos_error_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_T_des_xyz_rpy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_q_des_ik_ns_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_fk_q_des_xyz_rpy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_q_target_after_cbf_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_fk_q_target_xyz_rpy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_q_meas_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_joint_pos_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_ee_pos_track_err_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_force_tau_meas_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_force_tau_model_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_force_tau_ext_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_force_tau_comp_pub_;

    std::vector<int> velocity_sign_history_size_;
    double m_t{0.0};
    Eigen::Matrix4d desired_pose_init;

    Eigen::VectorXd freedrive_hold_q_;
    bool freedrive_hold_initialized_{false};

    mutable std::array<double, EE_FF_DIM> ee_ff_delta_above_since_{{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0}};

    Eigen::MatrixXd computeDoubleDiagDampingMatrix(
        const Eigen::MatrixXd& K,
        const Eigen::VectorXd& zeta,
        const Eigen::MatrixXd& Lambda_inv);

    JointState expandJointStateForGripper(const JointState& joint_state) const;

    bool loadFrictionCoulombFromFile(const std::string& file_path, int expected_dof);

    void publishRedundantArmDebug(const Eigen::VectorXd& T_des_xyz_rpy6,
                                  const Eigen::VectorXd& q_des_ik_ns,
                                  const Eigen::VectorXd& fk_q_des_xyz_rpy6,
                                  const Eigen::VectorXd& q_target_after_cbf,
                                  const Eigen::VectorXd& fk_q_target_xyz_rpy6,
                                  const Eigen::VectorXd& q_meas,
                                  const Eigen::VectorXd& joint_pos_error);

    Eigen::VectorXd computeFrictionCompensation(
        const Eigen::VectorXd& current_v,
        int model_dof,
        double position_error_norm = -1.0);

    void updateFilteredVelocity(const Eigen::VectorXd& current_v, int model_dof);
};

}  // namespace controllers
}  // namespace teleop_app
