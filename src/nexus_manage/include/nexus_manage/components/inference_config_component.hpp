/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: nexus_manage/include/nexus_manage/components/inference_config_component.hpp
 * @Description: 推理节点配置组件，负责调用InferenceConfig服务配置推理节点状态，包含重试机制
 * 
 * Copyright (c) 2025 by Pegasus, All Rights Reserved. 
 */

#ifndef __INFERENCE_CONFIG_COMPONENT_HPP__
#define __INFERENCE_CONFIG_COMPONENT_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <infra_msg/srv/inference_config.hpp>
#include <string>
#include <functional>
#include <memory>
#include <map>
#include <atomic>
#include <vector>


/*******************************************************************************
 * Structure definition
 ******************************************************************************/
namespace nexus_manage {
namespace components {

/**
  * @brief  推理配置响应结构体
  * @param  null
  * @retval null
  * @usage  
  */
struct InferenceConfigResponse {
    bool success;
    std::string error_message;
    uint8_t actual_state;
};

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  推理节点配置组件类
  * @param  null
  * @retval null
  * @usage  
  */
class InferenceConfigComponent {
public:
    using InferenceConfigSrv = infra_msg::srv::InferenceConfig;
    using ConfigCallback = std::function<void(const InferenceConfigResponse& response)>;
    
    /**
      * @brief  构造函数
      * @param  node  ROS2 node指针
      * @param  service_name  服务名称
      * @retval null
      * @usage  
      */
    InferenceConfigComponent(rclcpp::Node::SharedPtr node, const std::string& service_name);
    
    /**
      * @brief  异步配置推理节点状态
      * @param  state  目标状态（STATE_IDLE=0, STATE_INFERENCE=1, STATE_TAKEOVER=2）
      * @param  callback  回调函数
      * @param  timeout_sec  单次请求超时时间(秒，默认5.0秒)
      * @param  max_retries  最大重试次数（默认3次）
      * @param  retry_interval  重试间隔(秒，默认1.0秒)
      * @retval null
      * @usage  
      */
    void configureStateAsync(
        uint8_t state,
        ConfigCallback callback,
        double timeout_sec = 5.0,
        int max_retries = 3,
        double retry_interval = 1.0
    );
    
    /**
      * @brief  同步配置推理节点状态
      * @param  state  目标状态（STATE_IDLE=0, STATE_INFERENCE=1, STATE_TAKEOVER=2）
      * @param  response  响应数据
      * @param  timeout_sec  超时时间(秒)
      * @retval 是否成功
      * @usage  
      */
    bool configureStateSync(
        uint8_t state,
        InferenceConfigResponse& response,
        double timeout_sec = 5.0
    );
    
    /**
      * @brief  同步配置推理节点状态（简化版）
      * @param  state  目标状态（STATE_IDLE=0, STATE_INFERENCE=1, STATE_TAKEOVER=2）
      * @param  timeout_sec  超时时间(秒)
      * @retval 是否成功
      * @usage  
      */
    bool configureStateSync(
        uint8_t state,
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
    rclcpp::Client<InferenceConfigSrv>::SharedPtr client_;
    rclcpp::Logger logger_;
    std::vector<rclcpp::TimerBase::SharedPtr> retry_timers_;  // 保存重试定时器
    std::map<uint64_t, rclcpp::TimerBase::SharedPtr> timeout_timers_;  // 保存超时定时器
    std::atomic<uint64_t> request_counter_{0};  // 请求计数器
    
    /**
      * @brief  异步重试的内部实现
      * @param  state  目标状态
      * @param  callback  回调函数
      * @param  remaining_retries  剩余重试次数
      * @param  retry_interval  重试间隔
      * @param  timeout_sec  超时时间
      * @retval null
      * @usage  
      */
    void configureWithRetry(
        uint8_t state,
        ConfigCallback callback,
        int remaining_retries,
        double retry_interval,
        double timeout_sec
    );
    
    /**
      * @brief  创建服务请求
      * @param  state  目标状态
      * @retval 服务请求指针
      * @usage  
      */
    std::shared_ptr<InferenceConfigSrv::Request> createRequest(uint8_t state);
    
    /**
      * @brief  解析服务响应
      * @param  response  服务响应指针
      * @retval 解析后的响应结构体
      * @usage  
      */
    InferenceConfigResponse parseResponse(const std::shared_ptr<InferenceConfigSrv::Response>& response);
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace components
} // namespace nexus_manage

#endif
