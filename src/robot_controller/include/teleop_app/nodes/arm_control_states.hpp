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
