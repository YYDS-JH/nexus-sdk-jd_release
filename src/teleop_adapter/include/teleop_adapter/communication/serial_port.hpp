/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/communication/serial_port.hpp
 * @Description: 串口通讯层实现类，提供线程安全的串口读写能力
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef TELEOP_ADAPTER_SERIAL_PORT_HPP
#define TELEOP_ADAPTER_SERIAL_PORT_HPP

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/communication/communication_interface.hpp"
#include <string>
#include <cstdint>
#include <mutex>
#include <atomic>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  串口通讯实现类
  * @param  null
  * @retval null
  * @note   实现 CommunicationInterface 接口，提供线程安全的串口读写功能
  */
class SerialPort : public CommunicationInterface {
public:
    /**
      * @brief  串口配置参数结构体
      * @param  null
      * @retval null
      */
    struct Config {
        std::string port_name;      // 串口设备路径，如 "/dev/ttyUSB0"
        int baudrate;               // 波特率：9600, 115200, 460800 等
        int data_bits;              // 数据位：5, 6, 7, 8
        int stop_bits;              // 停止位：1, 2
        char parity;                // 校验位：'N'(无), 'E'(偶), 'O'(奇)
        int timeout_ms;             // 读取超时时间（毫秒），-1 表示不超时

        // 默认配置：8N1, 115200
        Config() 
            : port_name(""), 
              baudrate(115200), 
              data_bits(8), 
              stop_bits(1), 
              parity('N'), 
              timeout_ms(100) {}
        
        Config(const std::string& port, int baud = 115200) 
            : port_name(port), 
              baudrate(baud), 
              data_bits(8), 
              stop_bits(1), 
              parity('N'), 
              timeout_ms(100) {}
    };

    /**
      * @brief  构造函数
      * @param  null
      * @retval null
      */
    SerialPort();

    /**
      * @brief  析构函数，自动关闭串口
      * @param  null
      * @retval null
      */
    virtual ~SerialPort();

    // 禁止拷贝和赋值
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /**
      * @brief  设置配置并打开串口
      * @param  config 串口配置参数
      * @retval 成功返回 true，失败返回 false
      */
    bool open(const Config& config);

    /***************************************************************************
     * 实现 CommunicationInterface 接口
     **************************************************************************/

    /**
      * @brief  打开串口（使用已设置的配置）
      * @param  null
      * @retval 成功返回 true，失败返回 false
      */
    bool open() override;

    /**
      * @brief  关闭串口
      * @param  null
      * @retval null
      */
    void close() override;

    /**
      * @brief  检查串口是否已打开
      * @param  null
      * @retval 已打开返回 true，否则返回 false
      */
    bool isOpen() const override;

    /**
      * @brief  读取数据（非阻塞）
      * @param  buffer 数据缓冲区
      * @param  length 期望读取的字节数
      * @param  timeout_ms 超时时间（毫秒），-1 使用配置的默认超时，0 立即返回
      * @retval 实际读取的字节数，-1 表示错误
      */
    int read(uint8_t* buffer, size_t length, int timeout_ms = -1) override;

    /**
      * @brief  写入数据
      * @param  data 要写入的数据
      * @param  length 数据长度
      * @retval 实际写入的字节数，-1 表示错误
      */
    int write(const uint8_t* data, size_t length) override;

    /**
      * @brief  获取接收缓冲区中可读的字节数
      * @param  null
      * @retval 可读字节数，-1 表示错误
      */
    int getBytesAvailable() const override;

    /**
      * @brief  清空接收和发送缓冲区
      * @param  null
      * @retval null
      */
    void clearBuffer() override;

    /**
      * @brief  获取最后一次错误信息
      * @param  null
      * @retval 错误描述字符串
      */
    std::string getLastError() const override;

    /**
      * @brief  获取通讯类型标识
      * @param  null
      * @retval "Serial"
      */
    std::string getType() const override;

    /***************************************************************************
     * 串口特有方法
     **************************************************************************/

    /**
      * @brief  获取当前配置
      * @param  null
      * @retval 配置参数
      */
    Config getConfig() const;

protected:
    int fd_;                            // 文件描述符
    Config config_;                     // 串口配置
    mutable std::mutex mutex_;          // 互斥锁，保护并发访问
    std::atomic<bool> is_open_;         // 串口打开状态
    mutable std::string last_error_;    // 最后错误信息

    /**
      * @brief  配置串口参数
      * @param  null
      * @retval 成功返回 true，失败返回 false
      */
    bool configure();

    /**
      * @brief  设置错误信息
      * @param  error 错误描述
      * @retval null
      */
    void setError(const std::string& error);

    /**
      * @brief  将波特率转换为系统常量
      * @param  baudrate 波特率数值
      * @retval 系统波特率常量，0 表示不支持
      */
    int baudrateToConst(int baudrate) const;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_SERIAL_PORT_HPP
