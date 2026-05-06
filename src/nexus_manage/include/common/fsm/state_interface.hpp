/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: nexus_manage/include/common/fsm/state_interface.hpp
 * @Description: 状态机状态接口定义
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef __STATE_INTERFACE_HPP__
#define __STATE_INTERFACE_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <memory>
#include <string>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace robot_sdk {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  状态接口基类
  * @param  null
  * @retval null
  */
class IState {
public:
    /**
      * @brief  虚析构函数
      * @param  null
      * @retval null
      */
    virtual ~IState() = default;
    
    /**
      * @brief  获取状态名称
      * @param  null
      * @retval 状态名称
      */
    virtual std::string name() const = 0;
    
    /**
      * @brief  进入状态回调
      * @param  null
      * @retval null
      */
    virtual void on_entry() {}
    
    /**
      * @brief  退出状态回调
      * @param  null
      * @retval null
      */
    virtual void on_exit() {}
    
    /**
      * @brief  状态更新回调
      * @param  null
      * @retval null
      */
    virtual void on_update() {}
    
    /**
      * @brief  事件处理（返回目标状态名，空字符串表示不转换）
      * @param  event 事件名称
      * @retval 目标状态名称
      */
    virtual std::string handle_event(const std::string& event) = 0;
};

using StatePtr = std::shared_ptr<IState>;

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace robot_sdk

#endif // __STATE_INTERFACE_HPP__
