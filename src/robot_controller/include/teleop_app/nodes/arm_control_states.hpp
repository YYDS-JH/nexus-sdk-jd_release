#ifndef TELEOP_APP__NODES__ARM_CONTROL_STATES_HPP_
#define TELEOP_APP__NODES__ARM_CONTROL_STATES_HPP_

#include "teleop_app/common/fsm/state_interface.hpp"
#include "teleop_app/common/thread/circular_queue.hpp"
#include "teleop_app/controllers/data_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <vector>
#include <atomic>

namespace teleop_app {
namespace nodes {

// 前向声明
class ArmControlNode;

/**
 * @brief TELEOP_INIT 状态（原 STARTUP_CHECK）- 启动自检
 */
class StartupCheckState : public robot_sdk::IState {
public:
    explicit StartupCheckState(ArmControlNode* node);
    
    std::string name() const override { return "TELEOP_INIT"; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    ArmControlNode* node_;
    rclcpp::Time entry_time_;
};

/**
 * @brief TELEOP_RESET 状态（原 TELEOP_INIT）- 遥操初始化/复位
 */
class TeleopInitState : public robot_sdk::IState {
public:
    explicit TeleopInitState(ArmControlNode* node);
    ~TeleopInitState();
    
    std::string name() const override { return "TELEOP_RESET"; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    void controllerThreadFunc(int controller_index);
    
    ArmControlNode* node_;
    std::vector<std::shared_ptr<std::thread>> controller_threads_;
    std::vector<std::shared_ptr<CircularQueue<controllers::MotionPlanningResult>>> result_queues_;
    std::atomic<bool> threads_running_;
    bool completion_logged_ {false};
    bool reset_completed_logged_ {false};
};

/**
 * @brief TELEOP_RUN 状态（原 TELEOP）- 遥操运行
 */
class TeleopState : public robot_sdk::IState {
public:
    explicit TeleopState(ArmControlNode* node);
    ~TeleopState();
    
    std::string name() const override { return "TELEOP_RUN"; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    void controllerThreadFunc(int controller_index);
    void forcePublishThreadFunc(int controller_index);

    /**
     * @brief 从当前实际关节角同步 vqp 臂角参考与力反馈旋转角（进入 TELEOP_RUN 时调用）
     *
     * 读取 latest_joint_state_per_controller_ 中各臂当前关节角，
     * 更新 controller_params_ 中的 vqp_arm_angle_ref_joint_positions
     * 与 ee_ff_rotation_z_deg，并推送到运行时 IK 求解器。
     *
     * 设计目的：消除对 nexus_manage captureSlaveJointPositions() 时序的依赖，
     * 让 arm_control_node 自治决定臂角参考与力反馈坐标系。
     */
    void captureVqpRefFromCurrentJoints();

    ArmControlNode* node_;
    std::vector<std::shared_ptr<std::thread>> controller_threads_;
    std::vector<std::shared_ptr<std::thread>> force_publish_threads_;
    std::vector<std::shared_ptr<CircularQueue<controllers::MitControlCommand>>> result_queues_;
    std::atomic<bool> threads_running_;
};

/**
 * @brief TELEOP_FAULT 状态（原 FAULT）- 故障
 */
class FaultState : public robot_sdk::IState {
public:
    explicit FaultState(ArmControlNode* node);
    
    std::string name() const override { return "TELEOP_FAULT"; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    ArmControlNode* node_;
    rclcpp::Time last_warn_time_;
};

/**
 * @brief TELEOP_IDLE 状态 - 阻尼停稳（与 TELEOP_FAULT 相同控制输出）
 */
class TeleopIdleState : public robot_sdk::IState {
public:
    explicit TeleopIdleState(ArmControlNode* node);

    std::string name() const override { return "TELEOP_IDLE"; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    ArmControlNode* node_;
    rclcpp::Time last_warn_time_;
};

}  // namespace nodes
}  // namespace teleop_app

#endif  // TELEOP_APP__NODES__ARM_CONTROL_STATES_HPP_
