/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/adapters/device_adapter.hpp
 * @Description: 统一设备适配器基类，集成硬件通讯、协议实现、电机控制、传感器读取、设备管理等功能
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef TELEOP_ADAPTER_DEVICE_ADAPTER_HPP
#define TELEOP_ADAPTER_DEVICE_ADAPTER_HPP

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include "teleop_adapter/communication/communication_interface.hpp"

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Structure definition
 ******************************************************************************/

struct DeviceAdapterConfig {
    std::string device_name;
    int num_of_dofs;
    int feedback_rate;
    bool auto_enable;
    int seq_num_bits;
    int seq_window_size;
    
    DeviceAdapterConfig()
        : device_name(""),
          num_of_dofs(6),
          feedback_rate(400),
          auto_enable(false),
          seq_num_bits(16),
          seq_window_size(1024) {}
};

struct MotorState {
    double position;
    double velocity;
    double effort;
    double temperature;
    bool enabled;
    int error_code;
    
    MotorState()
        : position(0.0),
          velocity(0.0),
          effort(0.0),
          temperature(0.0),
          enabled(false),
          error_code(0) {}
};

struct MotorCommand {
    double kp;
    double position;
    double kd;
    double velocity;
    double torque;
    
    MotorCommand()
        : kp(0.0),
          position(0.0),
          kd(0.0),
          velocity(0.0),
          torque(0.0) {}
};

struct SensorData {
    struct ForceSensor {
        double fx, fy, fz;
        double tx, ty, tz;
    } force;
    
    struct IMU {
        double ax, ay, az;
        double gx, gy, gz;
        double qw, qx, qy, qz;
    } imu;
    
    bool force_valid;
    bool imu_valid;
    
    SensorData() : force_valid(false), imu_valid(false) {}
};

struct DeviceStatus {
    uint8_t connected;
    uint8_t status;               // 0:NORMAL, 1:WARNING, 2:FAULT, 3:OFFLINE, 4:INITIALIZING
    uint64_t total_packet_count;
    uint64_t total_packet_loss_count;
    float overall_packet_loss_rate;
    bool motors_enabled;
    int error_code;
    std::string error_message;
    bool initialized;
    
    DeviceStatus()
        : connected(0),
          status(4),
          total_packet_count(0),
          total_packet_loss_count(0),
          overall_packet_loss_rate(0.0f),
          motors_enabled(false),
          error_code(0),
          error_message(""),
          initialized(false) {}
};

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
 * @brief  统一设备适配器抽象基类
 * @note   提供完整的硬件设备操作接口。具体设备类型需要继承此类并实现纯虚函数。
 *         支持多种通讯方式：串口、以太网、CAN等（通过 CommunicationInterface 抽象）。
 */
class DeviceAdapter {
public:
    DeviceAdapter(const DeviceAdapterConfig& config,
                  std::unique_ptr<CommunicationInterface> comm_interface = nullptr);
    virtual ~DeviceAdapter();
    
    DeviceAdapter(const DeviceAdapter&) = delete;
    DeviceAdapter& operator=(const DeviceAdapter&) = delete;
    
    // 连接管理
    bool initialize();
    void shutdown();
    bool isConnected() const;
    
    // 电机控制（纯虚函数，需子类实现）
    virtual bool enableMotors() = 0;
    virtual bool disableMotors() = 0;
    virtual bool sendMotorCommands(const std::vector<MotorCommand>& commands) = 0;
    virtual bool readMotorStates(std::vector<MotorState>& states) = 0;
    
    // 传感器（默认不支持）
    virtual bool readSensorData(SensorData& sensor_data);

    // 外部力/力矩（基坐标系 [Fx,Fy,Fz,Tx,Ty,Tz]，默认不支持）
    virtual bool readExternalWrench(std::array<double, 6>& wrench);
    
    // 设备管理
    DeviceStatus getDeviceStatus() const;
    DeviceAdapterConfig getConfig() const;
    virtual bool clearErrors();
    int getNumDofs() const;
    std::string getDeviceName() const;
    void notifyReceivedFrame(uint32_t frame_seq);
    void enableExternalStats(bool enabled);

protected:
    // 子类必须实现的协议接口
    virtual bool deviceHandshake() = 0;
    virtual bool configureDevice() = 0;
    virtual void readThreadFunc() = 0;
    virtual void writeThreadFunc() = 0;
    
    // 子类可调用的辅助方法
    void setDeviceStatus(const DeviceStatus& status);
    void setError(int error_code, const std::string& error_message);
    void clearError();

    // 子类需要访问的成员
    DeviceAdapterConfig config_;
    std::unique_ptr<CommunicationInterface> comm_interface_;
    std::thread read_thread_;
    std::thread write_thread_;
    std::atomic<bool> thread_running_;
    mutable std::mutex status_mutex_;
    DeviceStatus device_status_;
    mutable std::mutex data_mutex_;

private:
    struct MonitorImpl;
    std::unique_ptr<MonitorImpl> monitor_impl_;
    void monitorThreadFunc();
};

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_DEVICE_ADAPTER_HPP
