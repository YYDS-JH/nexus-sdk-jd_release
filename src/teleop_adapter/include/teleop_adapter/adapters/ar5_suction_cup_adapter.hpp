/*
 * @FilePath: teleop_adapter/include/teleop_adapter/adapters/ar5_suction_cup_adapter.hpp
 * @Description: AR5机械臂适配器（带旋转吸盘），继承DeviceAdapter基类，
 *               8自由度（7 AR5关节 + 1 吸盘关节）
 *               吸盘关节位置值: >=0 → 角度45°, <0 → 角度135°
 *               |pos|<0.1 → 吸盘关, |pos|>0.5 → 吸盘开, 中间值保持上一状态
 *               角度旋转通过 PeripheralsClient HTTP API 控制，
 *               吸盘开关通过 robot.setDO(2,8,bool) 控制，
 *               两者由独立异步线程执行，状态上报最后命令值。
 *
 * Copyright (c) 2025 by Infra Embedded, All Rights Reserved.
 */

#ifndef TELEOP_ADAPTER_AR5_SUCTION_CUP_ADAPTER_HPP
#define TELEOP_ADAPTER_AR5_SUCTION_CUP_ADAPTER_HPP

#include "teleop_adapter/adapters/device_adapter.hpp"
#include <memory>
#include <string>
#include <array>
#include <vector>

namespace teleop_adapter {

#ifndef TELEOP_ADAPTER_AR5_ARM_DOF_DEFINED
#define TELEOP_ADAPTER_AR5_ARM_DOF_DEFINED
static constexpr int AR5_ARM_DOF = 7;
#endif
static constexpr int SUCTION_CUP_DOF = 1;
static constexpr int AR5_SUCTION_CUP_TOTAL_DOF = AR5_ARM_DOF + SUCTION_CUP_DOF;  // 8
static constexpr int IDX_SUCKER = 7;

class Ar5SuctionCupAdapter : public DeviceAdapter {
public:
    Ar5SuctionCupAdapter(const DeviceAdapterConfig& config,
                         const std::string& robot_ip,
                         const std::string& local_ip,
                         const std::string& peripheral_server_url,
                         const std::array<double, AR5_ARM_DOF>& init_joint_positions = {
                             0, 0.52359877559829887307710723054658312, 0,
                             1.0471975511965977461542144610931662, 0,
                             1.5707963267948966192313216916397514, 0},
                         double load_mass = 0,
                         const std::array<double, 3>& load_cog = {},
                         const std::array<double, 3>& load_inertia = {});
    ~Ar5SuctionCupAdapter() override;

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
    void suckerControlLoop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::array<double, AR5_ARM_DOF> computeMitTorque(
        const std::vector<MotorCommand>& commands,
        const std::array<double, AR5_ARM_DOF>& current_pos,
        const std::array<double, AR5_ARM_DOF>& current_vel) const;
};

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_AR5_SUCTION_CUP_ADAPTER_HPP
