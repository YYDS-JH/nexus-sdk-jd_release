/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: nexus_manage/include/nexus_manage/components/config_parser_component.hpp
 * @Description: 配置文件解析组件（使用ROS2参数）
 * 
 * Copyright (c) 2025 by Pegasus, All Rights Reserved. 
 */

#ifndef __CONFIG_PARSER_COMPONENT_HPP__
#define __CONFIG_PARSER_COMPONENT_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/logger.hpp>
#include <memory>
#include <string>
#include <vector>

#include "nexus_manage/components/end_effector_config.hpp"

namespace rclcpp {
class Node;
}


/*******************************************************************************
 * Class definition
 ******************************************************************************/
namespace nexus_manage {
namespace components {

/**
  * @brief  配置解析组件类
  * @param  null
  * @retval null
  * @usage  
  */
class ConfigParserComponent {
public:
    explicit ConfigParserComponent(
        std::shared_ptr<rclcpp::Node> node,
        rclcpp::Logger logger
    );
    ~ConfigParserComponent();
    
    /**
      * @brief  从ROS2参数加载配置
      * @param  null
      * @retval 是否成功
      * @usage  
      */
    bool loadFromParameters();
    
    /**
      * @brief  更新末端执行器的复位位置（从 ROS2 参数重新读取）
      * @param  null
      * @retval 是否成功
      * @usage  
      */
    bool updateResetPositions();
    
    const std::vector<EndEffectorConfig>& getEndEffectorConfigs() const;
    
    /**
      * @brief  获取YAML初始复位位置配置（只读，永远不会被修改）
      * @param  null
      * @retval 初始末端执行器配置列表引用
      * @usage  
      */
    const std::vector<EndEffectorConfig>& getInitialEndEffectorConfigs() const;
    
    std::string getRobotName() const;
    std::string getTeleopName() const;
    
    // 获取全局缩放因子（三轴独立）
    double getEEPosScalingFactorX() const;
    double getEEPosScalingFactorY() const;
    double getEEPosScalingFactorZ() const;
    double getEEVelScalingFactorX() const;
    double getEEVelScalingFactorY() const;
    double getEEVelScalingFactorZ() const;
    double getEERotScalingFactor() const;
    double getEEBaseFrameRotationZDeg() const;
    std::vector<double> getEERpyLimits() const;
    double getGripperScalingFactor() const;

    // 是否允许 marker_key 切换 gripper 值正负号（吸盘角度控制）
    bool getGripperSignFlipEnabled() const;

    // 获取状态机更新频率
    double getStateMachineRate() const;
    
    std::string getTopicRobotHumanData() const;
    std::string getTopicRobotCmd() const;
    std::string getTopicRobotControllerState() const;
    std::string getTopicTeleopHumanData() const;
    std::string getTopicTeleopPedalState() const;
    std::string getTopicTeleopGripperKeyState() const;
    std::string getTopicTeleopControllerState() const;
    std::string getTopicTeleopManagerNodeState() const;
    
    std::string getServiceRobotControllerConfig() const;
    std::string getServiceTeleopControllerConfig() const;
    
    // 模型推理相关 getter
    std::string getTopicInferenceCmd() const;
    std::string getServiceInferenceConfig() const;
    double getDoubleClickInterval() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace components
} // namespace nexus_manage

#endif
