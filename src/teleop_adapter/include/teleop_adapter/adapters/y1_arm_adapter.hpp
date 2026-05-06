/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/adapters/y1_arm_adapter.hpp
 * @Description: Y1机械臂适配器，继承DeviceAdapter基类，使用Y1 SDK实现机械臂控制
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef TELEOP_ADAPTER_Y1_ARM_ADAPTER_HPP
#define TELEOP_ADAPTER_Y1_ARM_ADAPTER_HPP

#include "teleop_adapter/adapters/device_adapter.hpp"
#include <memory>
#include <string>
#include <vector>

namespace teleop_adapter {

class Y1ArmAdapter : public DeviceAdapter {
public:
    Y1ArmAdapter(const DeviceAdapterConfig& config,
                 const std::string& can_id);
    ~Y1ArmAdapter() override;

    bool deviceHandshake() override;
    bool configureDevice() override;
    bool enableMotors() override;
    bool disableMotors() override;
    bool sendMotorCommands(const std::vector<MotorCommand>& commands) override;
    bool readMotorStates(std::vector<MotorState>& states) override;
    DeviceStatus getDeviceStatus() const;
    void readThreadFunc() override;
    void writeThreadFunc() override;
    void shutdown();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_Y1_ARM_ADAPTER_HPP
