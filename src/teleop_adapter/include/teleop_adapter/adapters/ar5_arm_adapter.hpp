/*
 * @Author: Infra Embedded
 * @Date: 2025-03-31 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/adapters/ar5_arm_adapter.hpp
 * @Description: AR5机械臂适配器，继承DeviceAdapter基类，封装Rokae SDK实时控制接口
 *
 * Copyright (c) 2025 by Infra Embedded, All Rights Reserved.
 */

#ifndef TELEOP_ADAPTER_AR5_ARM_ADAPTER_HPP
#define TELEOP_ADAPTER_AR5_ARM_ADAPTER_HPP

#include "teleop_adapter/adapters/device_adapter.hpp"
#include <memory>
#include <string>
#include <array>
#include <vector>

namespace teleop_adapter {

static constexpr int AR5_DOF = 7;

class Ar5ArmAdapter : public DeviceAdapter {
public:
    Ar5ArmAdapter(const DeviceAdapterConfig& config,
                  const std::string& robot_ip,
                  const std::string& local_ip,
                  const std::array<double, AR5_DOF>& init_joint_positions = {
                      0, 0.52359877559829887307710723054658312, 0, 1.0471975511965977461542144610931662, 0,
                      1.5707963267948966192313216916397514, 0},
                  double load_mass = 0,
                  const std::array<double, 3>& load_cog = {},
                  const std::array<double, 3>& load_inertia = {});
    ~Ar5ArmAdapter() override;

    bool deviceHandshake() override;
    bool configureDevice() override;
    bool enableMotors() override;
    bool disableMotors() override;
    bool sendMotorCommands(const std::vector<MotorCommand>& commands) override;
    bool readMotorStates(std::vector<MotorState>& states) override;
    bool readExternalWrench(std::array<double, 6>& wrench) override;
    DeviceStatus getDeviceStatus() const;
    void readThreadFunc() override;
    void writeThreadFunc() override;
    void shutdown();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::array<double, AR5_DOF> computeMitTorque(
        const std::vector<MotorCommand>& commands,
        const std::array<double, AR5_DOF>& current_pos,
        const std::array<double, AR5_DOF>& current_vel) const;
};

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_AR5_ARM_ADAPTER_HPP
