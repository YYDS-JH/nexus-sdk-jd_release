/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: nexus_manage/include/nexus_manage/components/controller_config_component.hpp
 * @Description: 控制器配置组件，负责调用ControllerConfig服务配置遥操作控制器，包含重试机制
 * 
 * Copyright (c) 2025 by Pegasus, All Rights Reserved. 
 */

#ifndef __CONTROLLER_CONFIG_COMPONENT_HPP__
#define __CONTROLLER_CONFIG_COMPONENT_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <infra_msg/srv/controller_config.hpp>
#include <infra_msg/msg/end_effector_config.hpp>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <map>
#include <atomic>


/*******************************************************************************
 * Structure definition
 ******************************************************************************/
namespace nexus_manage {
namespace components {

/**
  * @brief  控制器配置参数结构体
  * @param  null
  * @retval null
  * @usage  
  */
struct ControllerConfigParams {
    uint8_t command_type;                                    // 命令类型
    uint8_t controller_state;                                 // 控制器状态
    uint8_t control_mode;                                     // 运行模式
    std::vector<infra_msg::msg::EndEffectorConfig> end_effectors;  // 末端执行器配置
    std::string target_arm_name;                              // 目标机械臂名称
    float timeout;                                            // 超时时间(秒)
    
    // 默认构造函数
    ControllerConfigParams()
        : command_type(infra_msg::srv::ControllerConfig::Response::CMD_SET_CONTROLLER_STATE),
          controller_state(infra_msg::srv::ControllerConfig::Response::STATE_IDLE),
          control_mode(0),
          timeout(5.0f) {}
};

/**
  * @brief  控制器配置响应结构体
  * @param  null
  * @retval null
  * @usage  
  */
struct ControllerConfigResponse {
    bool success;
    int32_t error_code;
    std::string error_message;
    uint8_t actual_controller_state;
    uint8_t actual_control_mode;
    uint32_t response_time_ms;
};

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  控制器配置组件类
  * @param  null
  * @retval null
  * @usage  
  */
class ControllerConfigComponent {
public:
    using ControllerConfigSrv = infra_msg::srv::ControllerConfig;
    using ConfigCallback = std::function<void(const ControllerConfigResponse& response)>;
    
    /**
      * @brief  构造函数
      * @param  node  ROS2 node指针
      * @param  service_name  服务名称
      * @retval null
      * @usage  
      */
    ControllerConfigComponent(rclcpp::Node::SharedPtr node, const std::string& service_name);
    
    /**
      * @brief  异步调用配置服务（完整参数版本）
      * @param  params  控制器配置参数
      * @param  callback  回调函数
      * @param  timeout_sec  单次请求超时时间(秒，默认5.0秒)
      * @param  max_retries  最大重试次数（默认3次）
      * @param  retry_interval  重试间隔(秒，默认1.0秒)
      * @retval null
      * @usage  
      */
    void configureControllerAsync(
        const ControllerConfigParams& params,
        ConfigCallback callback,
        double timeout_sec = 5.0,
        int max_retries = 3,
        double retry_interval = 1.0
    );
    
    /**
      * @brief  异步调用配置服务（简化版 - 仅设置控制器状态）
      * @param  robot_name  机器人名称
      * @param  controller_state  控制器状态
      * @param  callback  回调函数
      * @param  timeout_sec  单次请求超时时间(秒，默认5.0秒)
      * @param  max_retries  最大重试次数（默认3次）
      * @param  retry_interval  重试间隔(秒，默认1.0秒)
      * @retval null
      * @usage  
      */
    void configureControllerStateAsync(
        const std::string& robot_name,
        uint8_t controller_state,
        ConfigCallback callback,
        double timeout_sec = 5.0,
        int max_retries = 3,
        double retry_interval = 1.0
    );
    
    /**
      * @brief  同步调用配置服务（完整参数版本）
      * @param  params  控制器配置参数
      * @param  response  响应数据
      * @param  timeout_sec  超时时间(秒)
      * @retval 是否成功
      * @usage  
      */
    bool configureControllerSync(
        const ControllerConfigParams& params,
        ControllerConfigResponse& response,
        double timeout_sec = 5.0
    );
    
    /**
      * @brief  同步调用配置服务（简化版 - 仅设置控制器状态）
      * @param  robot_name  机器人名称
      * @param  controller_state  控制器状态
      * @param  timeout_sec  超时时间(秒)
      * @retval 是否成功
      * @usage  
      */
    bool configureControllerStateSync(
        const std::string& robot_name,
        uint8_t controller_state,
        double timeout_sec = 5.0
    );
    
    /**
      * @brief  检查服务是否可用
      * @param  timeout_sec  等待超时时间(秒)
      * @retval 服务是否可用
      * @usage  
      */
    bool isServiceAvailable(double timeout_sec = 1.0);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<ControllerConfigSrv>::SharedPtr client_;
    rclcpp::Logger logger_;
    std::vector<rclcpp::TimerBase::SharedPtr> retry_timers_;  // 保存重试定时器
    std::map<uint64_t, rclcpp::TimerBase::SharedPtr> timeout_timers_;  // 保存超时定时器
    std::atomic<uint64_t> request_counter_{0};  // 请求计数器
    
    /**
      * @brief  异步重试的内部实现
      * @param  params  控制器配置参数
      * @param  callback  回调函数
      * @param  remaining_retries  剩余重试次数
      * @param  retry_interval  重试间隔
      * @retval null
      * @usage  
      */
    void configureWithRetry(
        const ControllerConfigParams& params,
        ConfigCallback callback,
        int remaining_retries,
        double retry_interval
    );
    
    /**
      * @brief  创建服务请求
      * @param  params  控制器配置参数
      * @retval 服务请求指针
      * @usage  
      */
    std::shared_ptr<ControllerConfigSrv::Request> createRequest(const ControllerConfigParams& params);
    
    /**
      * @brief  解析服务响应
      * @param  response  服务响应指针
      * @retval 解析后的响应结构体
      * @usage  
      */
    ControllerConfigResponse parseResponse(const std::shared_ptr<ControllerConfigSrv::Response>& response);
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace components
} // namespace nexus_manage

#endif
