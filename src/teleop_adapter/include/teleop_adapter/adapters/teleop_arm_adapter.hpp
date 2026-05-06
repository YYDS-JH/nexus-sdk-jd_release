/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/adapters/teleop_arm_adapter.hpp
 * @Description: Teleop Arm 机械臂设备适配器，继承 DeviceAdapter 基类，实现 Teleop Arm 特定的串口通讯协议
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

#ifndef __TELEOP_ARM_ADAPTER_HPP__
#define __TELEOP_ARM_ADAPTER_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/

#include "teleop_adapter/adapters/device_adapter.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace teleop_adapter {

/*******************************************************************************
 * Protocol (motor mode values used in public API)
 ******************************************************************************/

static constexpr uint8_t MOTOR_MODE_DISABLE = 0x00;
static constexpr uint8_t MOTOR_MODE_ENABLE = 0x0A;

/**
 * @brief 单电机反馈数据结构 (10 bytes per motor)
 */
struct MotorFeedbackData {
    uint8_t   work_status;
    uint16_t  current_pos;
    uint16_t  current_vel;
    uint16_t  current_torque;
    uint8_t   temperature;
    uint8_t   error_code;
} __attribute__((packed));

/**
 * @brief 外设状态结构体 (0x9D响应帧)
 */
struct PeripheralState {
    uint8_t led_status;
    uint8_t button_status;
    uint16_t joystick_x;
    uint16_t joystick_y;

    std::chrono::steady_clock::time_point timestamp;

    uint8_t getRedLedMode() const { return (led_status >> 0) & 0x03; }
    uint8_t getGreenLedMode() const { return (led_status >> 2) & 0x03; }
    uint8_t getBlueLedMode() const { return (led_status >> 4) & 0x03; }

    bool isButtonPressed(int button_idx) const {
        if (button_idx < 0 || button_idx > 7) return false;
        return (button_status >> button_idx) & 0x01;
    }

    int16_t getJoystickXNormalized() const {
        return static_cast<int16_t>(std::round((static_cast<double>(joystick_x) - 2048.0) / 20.48));
    }
    int16_t getJoystickYNormalized() const {
        return static_cast<int16_t>(std::round((static_cast<double>(joystick_y) - 2048.0) / 20.48));
    }
};

/**
 * @brief Teleop Arm 机械臂适配器
 */
class TeleopArmAdapter : public DeviceAdapter {
public:
    TeleopArmAdapter(const DeviceAdapterConfig& config,
                    std::unique_ptr<CommunicationInterface> comm_interface);

    virtual ~TeleopArmAdapter();

    bool enableMotors() override;
    bool disableMotors() override;
    bool sendMotorCommands(const std::vector<MotorCommand>& commands, uint8_t mode);
    bool sendMotorCommands(const std::vector<MotorCommand>& commands) override;
    bool readMotorStates(std::vector<MotorState>& states) override;
    bool deviceHandshake() override;
    bool configureDevice() override;
    void readThreadFunc() override;
    void writeThreadFunc() override;

    bool setMotorZero(int timeout_ms = 1000);
    bool clearDeviceErrors(int timeout_ms = 1000);
    bool queryPacketLoss(std::vector<float>& packet_loss_rates, int timeout_ms = 1000);
    bool setCommFrequency(int frequency_hz, int timeout_ms = 1000);
    bool setPeripheralState(uint8_t led_control, int timeout_ms = 1000);
    bool getPeripheralState(PeripheralState& peripheral_state) const;

    std::vector<MotorFeedbackData> getMotorFeedback() const;
    bool hasErrors() const;

private:
    struct Impl;
    friend struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace teleop_adapter

#endif // __TELEOP_ARM_ADAPTER_HPP__
