/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/communication/communication_interface.hpp
 * @Description: 通用通讯接口抽象基类，支持串口、以太网、CAN等各类硬件通讯方式
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef TELEOP_ADAPTER_COMMUNICATION_INTERFACE_HPP
#define TELEOP_ADAPTER_COMMUNICATION_INTERFACE_HPP

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <string>
#include <cstdint>
#include <cstddef>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  通讯接口抽象基类
  * @param  null
  * @retval null
  * @note   定义统一的通讯接口，支持多种硬件传输层：串口、以太网、CAN总线、USB等
  */
class CommunicationInterface {
public:
    /**
      * @brief  虚析构函数
      * @param  null
      * @retval null
      */
    virtual ~CommunicationInterface() = default;

    /**
      * @brief  打开通讯端口
      * @param  null
      * @retval 成功返回 true，失败返回 false
      */
    virtual bool open() = 0;

    /**
      * @brief  关闭通讯端口
      * @param  null
      * @retval null
      */
    virtual void close() = 0;

    /**
      * @brief  检查通讯端口是否已打开
      * @param  null
      * @retval 已打开返回 true，否则返回 false
      */
    virtual bool isOpen() const = 0;

    /**
      * @brief  读取数据
      * @param  buffer 数据缓冲区
      * @param  length 期望读取的字节数
      * @param  timeout_ms 超时时间（毫秒），-1 使用默认超时
      * @retval 实际读取的字节数，-1 表示错误
      */
    virtual int read(uint8_t* buffer, size_t length, int timeout_ms = -1) = 0;

    /**
      * @brief  写入数据
      * @param  data 要写入的数据
      * @param  length 数据长度
      * @retval 实际写入的字节数，-1 表示错误
      */
    virtual int write(const uint8_t* data, size_t length) = 0;

    /**
      * @brief  获取可读字节数
      * @param  null
      * @retval 可读字节数，-1 表示错误或不支持
      */
    virtual int getBytesAvailable() const = 0;

    /**
      * @brief  清空接收和发送缓冲区
      * @param  null
      * @retval null
      */
    virtual void clearBuffer() = 0;

    /**
      * @brief  获取最后一次错误信息
      * @param  null
      * @retval 错误描述字符串
      */
    virtual std::string getLastError() const = 0;

    /**
      * @brief  获取通讯类型标识
      * @param  null
      * @retval 通讯类型字符串（如 "Serial", "Ethernet", "CAN"）
      */
    virtual std::string getType() const = 0;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_COMMUNICATION_INTERFACE_HPP
