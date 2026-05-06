/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: nexus_manage/include/nexus_manage/nodes/teleop_manager_node.hpp
 * @Description: 遥操作管理器节点，ROS2通信层，协调各组件和状态机运行
 * 
 * Copyright (c) 2025 by Pegasus, All Rights Reserved. 
 */

#ifndef __TELEOP_MANAGER_NODE_HPP__
#define __TELEOP_MANAGER_NODE_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>

#include <infra_msg/msg/arm_control_state.hpp>
#include <infra_msg/msg/human_data_interface.hpp>
#include <infra_msg/msg/robot_ee_cmd.hpp>

#include <cstdint>
#include <memory>
#include <string>


/*******************************************************************************
 * Class definition
 ******************************************************************************/
namespace nexus_manage {

namespace components {
class ConfigParserComponent;
class KeyDetectorComponent;
class ControllerConfigComponent;
class CommandCalculatorComponent;
class InferenceConfigComponent;
} // namespace components

namespace states {
class BootSelfCheckState;
class IdleState;
class ResetState;
class ResetCompleteState;
class PositionHoldState;
class TeleopRunningState;
class TeleopPausedState;
class FaultState;
class ModelInferenceState;
} // namespace states

/**
  * @brief  遥操作管理器节点类
  * @param  null
  * @retval null
  * @usage  
  */
class TeleopManagerNode : public rclcpp::Node {
public:
    /**
      * @brief  构造函数
      * @param  options  ROS2节点选项
      * @retval null
      * @usage  
      */
    explicit TeleopManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    /**
      * @brief  析构函数
      * @param  null
      * @retval null
      * @usage  
      */
    ~TeleopManagerNode();
    
    /**
      * @brief  启动节点（在构造函数之后调用）
      * @param  null
      * @retval null
      * @usage  
      */
    void start();
    
    /**
      * @brief  获取配置解析器组件
      * @param  null
      * @retval 配置解析器组件指针
      * @usage  
      */
    std::shared_ptr<components::ConfigParserComponent> getConfigParser();
    
    /**
      * @brief  获取按键检测组件
      * @param  null
      * @retval 按键检测组件指针
      * @usage  
      */
    std::shared_ptr<components::KeyDetectorComponent> getKeyDetector();
    
    /**
      * @brief  获取从臂控制器配置组件
      * @param  null
      * @retval 从臂控制器配置组件指针
      * @usage  
      */
    std::shared_ptr<components::ControllerConfigComponent> getRobotControllerConfig();
    
    /**
      * @brief  获取主臂控制器配置组件
      * @param  null
      * @retval 主臂控制器配置组件指针
      * @usage  
      */
    std::shared_ptr<components::ControllerConfigComponent> getTeleopControllerConfig();
    
    /**
      * @brief  获取指令计算组件
      * @param  null
      * @retval 指令计算组件指针
      * @usage  
      */
    std::shared_ptr<components::CommandCalculatorComponent> getCommandCalculator();
    
    /**
      * @brief  获取最新的主臂人体数据（线程安全）
      * @param  null
      * @retval 主臂人体数据拷贝
      * @usage  
      */
    infra_msg::msg::HumanDataInterface getLatestTeleopHumanData() const;
    
    /**
      * @brief  获取最新的从臂人体数据（线程安全）
      * @param  null
      * @retval 从臂人体数据拷贝
      * @usage  
      */
    infra_msg::msg::HumanDataInterface getLatestRobotHumanData() const;
    
    /**
      * @brief  获取最新的从臂控制器状态（线程安全）
      * @param  null
      * @retval 从臂控制器状态拷贝
      * @usage  
      */
    infra_msg::msg::ArmControlState getLatestRobotControllerState() const;
    
    /**
      * @brief  获取最新的主臂控制器状态（线程安全）
      * @param  null
      * @retval 主臂控制器状态拷贝
      * @usage  
      */
    infra_msg::msg::ArmControlState getLatestTeleopControllerState() const;
    
    /**
      * @brief  触发状态机事件
      * @param  event  事件名称
      * @retval null
      * @usage  
      */
    void triggerEvent(const std::string& event);
    
    /**
      * @brief  发布机器人指令
      * @param  cmd  机器人末端执行器指令
      * @retval null
      * @usage  
      */
    void publishRobotCmd(const infra_msg::msg::RobotEECmd& cmd);
    
    /**
      * @brief  发布状态信息（状态变更时调用）
      * @param  state  状态枚举值
      * @param  state_name  状态名称
      * @param  error_code  错误码（仅FAULT状态有效）
      * @param  error_message  错误信息（仅FAULT状态有效）
      * @retval null
      * @usage  
      */
    void publishState(uint8_t state, const std::string& state_name, 
                      int32_t error_code = 0, const std::string& error_message = "");
    
    /**
      * @brief  更新遥操作指令（由状态机调用）
      * @param  null
      * @retval null
      * @usage  
      */
    void updateTeleopCommand();
    
    /**
      * @brief  获取推理节点配置组件
      * @param  null
      * @retval 推理节点配置组件指针
      * @usage  
      */
    std::shared_ptr<components::InferenceConfigComponent> getInferenceConfig();
    
    /**
      * @brief  转发推理指令给机器人
      * @param  null
      * @retval null
      * @usage  由 ModelInferenceState::on_update() 调用
      */
    void forwardInferenceCommand();
    
    /**
      * @brief  重置推理指令标志（退出推理状态时调用）
      * @param  null
      * @retval null
      * @usage  由 ModelInferenceState::on_exit() 调用，避免下次进入推理状态时使用残留指令
      */
    void resetInferenceFlags();
    
    /**
      * @brief  从节点参数同步缩放因子到 command_calculator
      * @param  null
      * @retval null
      * @usage  由状态机在进入 TeleopRunning 等状态时调用，确保参数一致性
      */
    void syncScalingParamsFromNodeParams();
    
    /**
      * @brief  获取最新推理指令（线程安全）
      * @param  null
      * @retval 推理指令拷贝
      * @usage  
      */
    infra_msg::msg::RobotEECmd getLatestInferenceCmd() const;
    
    /**
      * @brief  检查是否有新的推理指令
      * @param  null
      * @retval 是否有新指令
      * @usage  
      */
    bool hasNewInferenceCmd() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    friend class states::BootSelfCheckState;
    friend class states::IdleState;
    friend class states::ResetState;
    friend class states::ResetCompleteState;
    friend class states::PositionHoldState;
    friend class states::TeleopRunningState;
    friend class states::TeleopPausedState;
    friend class states::FaultState;
    friend class states::ModelInferenceState;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace nexus_manage

#endif
