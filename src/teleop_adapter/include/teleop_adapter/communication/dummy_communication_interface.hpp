/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/communication/dummy_communication_interface.hpp
 * @Description: 虚拟通信接口（用于不需要串口通信的适配器，如Y1Adapter）
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef TELEOP_ADAPTER_DUMMY_COMMUNICATION_INTERFACE_HPP
#define TELEOP_ADAPTER_DUMMY_COMMUNICATION_INTERFACE_HPP

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/communication/communication_interface.hpp"

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  虚拟通信接口类
  * @param  null
  * @retval null
  * @note   用于不需要串口通信的适配器，如Y1Adapter
  */
class DummyCommunicationInterface : public CommunicationInterface {
public:
    /**
      * @brief  默认构造函数
      * @param  null
      * @retval null
      */
    DummyCommunicationInterface() = default;
    
    /**
      * @brief  析构函数
      * @param  null
      * @retval null
      */
    ~DummyCommunicationInterface() override = default;

    /**
      * @brief  打开通讯端口（虚拟实现，总是返回true）
      * @param  null
      * @retval true
      */
    bool open() override { return true; }
    
    /**
      * @brief  关闭通讯端口（虚拟实现，无操作）
      * @param  null
      * @retval null
      */
    void close() override {}
    
    /**
      * @brief  检查通讯端口是否已打开（虚拟实现，总是返回true）
      * @param  null
      * @retval true
      */
    bool isOpen() const override { return true; }
    
    /**
      * @brief  写入数据（虚拟实现，返回写入长度）
      * @param  data 要写入的数据
      * @param  length 数据长度
      * @retval 数据长度
      */
    int write(const uint8_t* data, size_t length) override { 
        (void)data; (void)length;
        return static_cast<int>(length); 
    }
    
    /**
      * @brief  读取数据（虚拟实现，返回0）
      * @param  buffer 数据缓冲区
      * @param  length 期望读取的字节数
      * @param  timeout_ms 超时时间
      * @retval 0
      */
    int read(uint8_t* buffer, size_t length, int timeout_ms = -1) override { 
        (void)buffer; (void)length; (void)timeout_ms;
        return 0; 
    }
    
    /**
      * @brief  获取可读字节数（虚拟实现，返回0）
      * @param  null
      * @retval 0
      */
    int getBytesAvailable() const override { return 0; }
    
    /**
      * @brief  清空缓冲区（虚拟实现，无操作）
      * @param  null
      * @retval null
      */
    void clearBuffer() override {}
    
    /**
      * @brief  获取最后一次错误信息（虚拟实现，返回空字符串）
      * @param  null
      * @retval 空字符串
      */
    std::string getLastError() const override { return ""; }
    
    /**
      * @brief  获取通讯类型标识
      * @param  null
      * @retval "dummy"
      */
    std::string getType() const override { return "dummy"; }
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_DUMMY_COMMUNICATION_INTERFACE_HPP
