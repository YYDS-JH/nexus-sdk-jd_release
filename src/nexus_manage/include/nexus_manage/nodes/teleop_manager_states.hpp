/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: nexus_manage/include/nexus_manage/nodes/teleop_manager_states.hpp
 * @Description: 遥操作管理器状态机定义，包含9个状态：BootSelfCheck, Idle, Reset, ResetComplete, PositionHold, TeleopRunning, TeleopPaused, Fault, ModelInference
 * 
 * Copyright (c) 2025 by Pegasus, All Rights Reserved. 
 */

#ifndef __TELEOP_MANAGER_STATES_HPP__
#define __TELEOP_MANAGER_STATES_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <common/fsm/state_interface.hpp>
#include <string>
#include <memory>
#include <vector>


/*******************************************************************************
 * Structure definition
 ******************************************************************************/

// 前向声明
namespace nexus_manage {
class TeleopManagerNode;
}

namespace nexus_manage {
namespace states {

/**
  * @brief  状态机事件名称常量
  * @param  null
  * @retval null
  * @usage  
  */
namespace Events {
    const std::string BOOT_COMPLETE = "boot_complete";
    const std::string RESET_REQUEST = "reset_request";
    const std::string RESET_SUCCESS = "reset_success";
    const std::string RESET_ABORT = "reset_abort";
    const std::string RESET_FAILURE = "reset_failure";
    const std::string TELEOP_START = "teleop_start";
    const std::string TELEOP_PAUSE = "teleop_pause";
    const std::string TELEOP_RESUME = "teleop_resume";
    const std::string TELEOP_STOP = "teleop_stop";
    const std::string POSITION_HOLD_EXIT = "position_hold_exit";
    const std::string SAFETY_STOP = "safety_stop";
    const std::string FAULT_CLEAR = "fault_clear";
    const std::string INFERENCE_ENTER = "inference_enter";
    const std::string INFERENCE_EXIT = "inference_exit";
    const std::string ROBOT_SWITCH = "robot_switch";
    const std::string PLACEMENT_START = "placement_start";
    const std::string PLACEMENT_CONFIRM = "placement_confirm";
    const std::string PLACEMENT_ABORT = "placement_abort";
    const std::string PLACEMENT_FAILURE = "placement_failure";
    const std::string SCHEDULER_MODE_IDLE = "scheduler_mode_idle";
}

/**
  * @brief  状态名称常量
  * @param  null
  * @retval null
  * @usage  
  */
namespace StateNames {
    const std::string BOOT_SELF_CHECK = "BootSelfCheck";
    const std::string IDLE = "Idle";
    const std::string RESET = "Reset";
    const std::string RESET_COMPLETE = "ResetComplete";
    const std::string POSITION_HOLD = "PositionHold";
    const std::string TELEOP_RUNNING = "TeleopRunning";
    const std::string TELEOP_PAUSED = "TeleopPaused";
    const std::string FAULT = "Fault";
    const std::string MODEL_INFERENCE = "ModelInference";
    const std::string PLACEMENT = "Placement";
    const std::string AWAIT_CONTROL_RELEASE = "AwaitControlRelease";
}

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  BootSelfCheck 状态：启动自检
  * @param  null
  * @retval null
  * @usage  
  */
class BootSelfCheckState : public robot_sdk::IState {
public:
    explicit BootSelfCheckState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::BOOT_SELF_CHECK; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
    rclcpp::Time entry_time_;
    bool self_check_passed_{false};
    bool capture_done_{false};
    bool idle_sent_for_switch_{false};
    bool phase2_started_{false};
    
    // 自检步骤函数
    bool checkConfiguration();
    bool checkControllerServices();
    bool checkTeleopControllerService();
    bool checkTeleopData();
    bool checkControllerStates();
    bool checkHumanDataMessages();
    bool checkEndEffectorData();
    void captureSlaveJointPositions();
};

/**
  * @brief  Idle 状态：空闲
  * @param  null
  * @retval null
  * @usage  
  */
class IdleState : public robot_sdk::IState {
public:
    explicit IdleState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::IDLE; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
};

/**
  * @brief  Reset 状态：复位中
  * @param  null
  * @retval null
  * @usage  
  */
class ResetState : public robot_sdk::IState {
public:
    explicit ResetState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::RESET; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
    bool reset_request_sent_{false};          // 复位请求已发送标志
    bool running_request_sent_{false};        // 运行请求已发送标志
    rclcpp::Time reset_start_time_;
    
    // 辅助方法
    void sendResetRequest();
    void sendRunningRequest();
    bool joint_pos_check(const std::vector<double>& current_pos,
                         const std::vector<double>& target_pos,
                         double tolerance) const;
    bool checkResetPositionReached();  // 检查是否到达复位位置
};

/**
  * @brief  ResetComplete 状态：复位完成
  * @param  null
  * @retval null
  * @usage  
  */
class ResetCompleteState : public robot_sdk::IState {
public:
    explicit ResetCompleteState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::RESET_COMPLETE; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
};

/**
  * @brief  PositionHold 状态：位置保持
  * @param  null
  * @retval null
  * @usage  
  */
class PositionHoldState : public robot_sdk::IState {
public:
    explicit PositionHoldState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::POSITION_HOLD; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
};

/**
  * @brief  TeleopRunning 状态：遥操作运行中
  * @param  null
  * @retval null
  * @usage  
  */
class TeleopRunningState : public robot_sdk::IState {
public:
    explicit TeleopRunningState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::TELEOP_RUNNING; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
};

/**
  * @brief  TeleopPaused 状态：遥操作暂停
  * @param  null
  * @retval null
  * @usage  
  */
class TeleopPausedState : public robot_sdk::IState {
public:
    explicit TeleopPausedState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::TELEOP_PAUSED; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
};

/**
  * @brief  Fault 状态：故障
  * @param  null
  * @retval null
  * @usage  
  */
class FaultState : public robot_sdk::IState {
public:
    explicit FaultState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::FAULT; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
    std::string fault_message_;
};

/**
  * @brief  ModelInference 状态：模型推理
  * @param  null
  * @retval null
  * @usage  在遥操作运行状态下单击接管按键(take_over_key)进入/退出，转发推理节点的指令给机器人
  */
class ModelInferenceState : public robot_sdk::IState {
public:
    explicit ModelInferenceState(std::shared_ptr<TeleopManagerNode> node);
    
    std::string name() const override { return StateNames::MODEL_INFERENCE; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
};

/**
  * @brief  落格确认后等待调度 mode=idle 释放 RCI
  */
class AwaitControlReleaseState : public robot_sdk::IState {
public:
    explicit AwaitControlReleaseState(std::shared_ptr<TeleopManagerNode> node);

    std::string name() const override { return StateNames::AWAIT_CONTROL_RELEASE; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    std::shared_ptr<TeleopManagerNode> node_;
};

/**
  * @brief  Placement 状态：落格流程（移动→释放吸盘→等待确认）
  * @usage  PositionHold 下长按 take_over_key 进入；可配置自动 MoveJ；data_collect_key 确认后进 Idle
  */
class PlacementState : public robot_sdk::IState {
public:
    explicit PlacementState(std::shared_ptr<TeleopManagerNode> node);

    std::string name() const override { return StateNames::PLACEMENT; }
    void on_entry() override;
    void on_exit() override;
    void on_update() override;
    std::string handle_event(const std::string& event) override;

private:
    enum class Phase {
        MOVING,
        RELEASING,
        AWAIT_CONFIRM
    };

    std::shared_ptr<TeleopManagerNode> node_;
    Phase phase_{Phase::MOVING};
    bool move_request_sent_{false};
    rclcpp::Time phase_start_time_;
    rclcpp::Time entry_time_;

    void publishPhaseState(const std::string& phase_name);
    void sendPlacementMoveRequest();
    void sendSlaveRunningRequest();
    void pausePlacementMove();
    void logPlacementProgressThrottled();
    bool checkPlacementPositionReached() const;
    bool jointPosCheck(const std::vector<double>& current_pos,
                       const std::vector<double>& target_pos,
                       double tolerance) const;
    static const char* phaseToString(Phase phase);

    rclcpp::Time last_progress_log_time_;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace states
} // namespace nexus_manage

#endif
