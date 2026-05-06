/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: human_data/include/human_data/common/fsm/state_machine.hpp
 * @Description: 状态机接口定义
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef __STATE_MACHINE_HPP__
#define __STATE_MACHINE_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "state_interface.hpp"
#include <memory>
#include <unordered_map>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace robot_sdk {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  状态机抽象基类
  * @param  null
  * @retval null
  */
class StateMachine {
public:
    /**
      * @brief  虚析构函数
      * @param  null
      * @retval null
      */
    virtual ~StateMachine() = default;
    
    /**
      * @brief  初始化状态机
      * @param  initial_state 初始状态名称
      * @retval null
      */
    virtual void initialize(const std::string& initial_state) = 0;
    
    /**
      * @brief  处理事件
      * @param  event 事件名称
      * @retval null
      */
    virtual void process_event(const std::string& event) = 0;
    
    /**
      * @brief  状态更新
      * @param  null
      * @retval null
      */
    virtual void update() = 0;
    
    /**
      * @brief  获取当前状态
      * @param  null
      * @retval 当前状态名称
      */
    virtual std::string current_state() const = 0;
    
    /**
      * @brief  注册状态
      * @param  name 状态名称
      * @param  state 状态指针
      * @retval null
      */
    virtual void register_state(const std::string& name, StatePtr state) = 0;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

/**
  * @brief  创建状态机实例
  * @param  null
  * @retval 状态机指针
  */
std::unique_ptr<StateMachine> create_state_machine();

} // namespace robot_sdk

#endif // __STATE_MACHINE_HPP__
