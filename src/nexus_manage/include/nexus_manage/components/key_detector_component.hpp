/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: nexus_manage/include/nexus_manage/components/key_detector_component.hpp
 * @Description: 按键检测组件，负责监测脚踏板和夹爪按键状态，实现按键聚合、长按检测和边缘检测功能
 * 
 * Copyright (c) 2025 by Pegasus, All Rights Reserved. 
 */

#ifndef __KEY_DETECTOR_COMPONENT_HPP__
#define __KEY_DETECTOR_COMPONENT_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/logger.hpp>
#include <infra_msg/msg/teleop_pedal_state.hpp>
#include <infra_msg/msg/teleop_gripper_key_state.hpp>
#include <memory>
#include <string>


/*******************************************************************************
 * Structure definition
 ******************************************************************************/
namespace nexus_manage {
namespace components {

/**
  * @brief  聚合后的按键状态结构体
  * @param  null
  * @retval null
  * @usage  
  */
struct AggregatedKeyState {
    // 脚踏板按键
    bool reset_pedal{false};         // REST脚踏板
    bool suspend_pedal{false};       // suspend脚踏板
    bool increment_pedal{false};     // 增量脚踏板
    bool scaling_plus_pedal{false};  // +scaling脚踏板
    bool scaling_minus_pedal{false}; // -scaling脚踏板
    
    // 夹爪按键（从GripperKeyState聚合）
    bool teleop_key{false};          // 遥操按键
    bool data_collect_key{false};    // 数采按键
    bool marker_key{false};          // 打标按键
    bool safety_key{false};          // 安全按键
    bool take_over_key{false};       // 接管按键
    
    // 长按持续时间(秒)
    double teleop_duration{0.0};
};

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  按键检测组件类
  * @param  null
  * @retval null
  * @usage  
  */
class KeyDetectorComponent {
public:
    /**
      * @brief  构造函数
      * @param  logger  ROS2 logger
      * @retval null
      * @usage  
      */
    explicit KeyDetectorComponent(rclcpp::Logger logger);
    ~KeyDetectorComponent();
    
    /**
      * @brief  更新脚踏板状态
      * @param  msg  脚踏板状态消息
      * @retval null
      * @usage  
      */
    void updatePedalState(const infra_msg::msg::TeleopPedalState::SharedPtr msg);
    
    /**
      * @brief  更新虚拟脚踏板状态（来自Web端模拟）
      * @param  msg  虚拟脚踏板状态消息
      * @retval null
      * @usage  与物理踏板做 OR 合并
      */
    void updateVirtualPedalState(const infra_msg::msg::TeleopPedalState::SharedPtr msg);
    
    /**
      * @brief  更新夹爪按键状态
      * @param  msg  夹爪按键状态消息
      * @retval null
      * @usage  
      */
    void updateGripperKeyState(const infra_msg::msg::TeleopGripperKeyState::SharedPtr msg);
    
    /**
      * @brief  获取聚合后的按键状态
      * @param  null
      * @retval 聚合后的按键状态引用
      * @usage  
      */
    AggregatedKeyState getAggregatedState() const;
    
    /**
      * @brief  检测按键边缘（按下事件）
      * @param  key_name  按键名称
      * @retval 如果按键从松开变为按下返回true
      * @usage  
      */
    bool detectRisingEdge(const std::string& key_name);
    
    /**
      * @brief  检测按键边缘（松开事件）
      * @param  key_name  按键名称
      * @retval 如果按键从按下变为松开返回true
      * @usage  
      */
    bool detectFallingEdge(const std::string& key_name);
    
    /**
      * @brief  检测按键当前是否按下
      * @param  key_name  按键名称
      * @retval 如果按键当前为按下状态返回true
      * @usage  
      */
    bool isKeyPressed(const std::string& key_name) const;
    
    /**
      * @brief  检测按键长按
      * @param  key_name  按键名称
      * @param  duration  长按阈值（秒）
      * @retval 如果按键按下时间超过阈值返回true
      * @usage  
      */
    bool detectLongPress(const std::string& key_name, double duration) const;
    
    /**
      * @brief  更新长按检测（需要周期调用）
      * @param  dt  时间间隔(秒)
      * @retval null
      * @usage  
      */
    void updateLongPress(double dt);
    
    /**
      * @brief  检测按键双击
      * @param  key_name  按键名称
      * @param  interval  双击判定时间间隔（秒），两次上升沿间隔小于此值判定为双击
      * @retval 如果检测到双击返回true
      * @usage  
      */
    bool detectDoubleClick(const std::string& key_name, double interval);
    
    /**
      * @brief  清除所有边缘检测标志位（上升沿、下降沿、双击）
      * @param  null
      * @retval null
      * @usage  状态切换时调用，防止残留标志在新状态中误触发
      */
    void clearAllEdgeFlags();

public:
    struct Impl;
private:
    std::unique_ptr<Impl> impl_;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace components
} // namespace nexus_manage

#endif
