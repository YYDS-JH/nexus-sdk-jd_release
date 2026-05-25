#ifndef PERIPHERALS_REMOTE_CLIENT_H
#define PERIPHERALS_REMOTE_CLIENT_H

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <map>        // 添加：用于 std::map
#include <cstdint>    // 添加：用于 uint8_t

namespace peripherals_remote {

// 模拟量信息
struct AnalogInfo {
    int channel;
    float voltage;
    float current;
    int raw_value;
};

// 数字IO信息
struct DigitalInfo {
    int board;
    int port;
    int value;
    int channel;      // 添加：用于通道号
};

// BMS基本信息
struct BMSBasicInfo {
    float total_voltage;
    float current;
    float remaining_capacity;
    float nominal_capacity;
    int cycle_count;
    int rsoc;
    int fet_control;
    int protection_status;
    int cell_count;
    int temp_count;   // 添加：温度数量
    std::vector<float> temperatures;
};

// BMS电池电压信息
struct BMSCellVoltageInfo {
    int cell_count;
    std::vector<float> voltages;
    float min_voltage;
    float max_voltage;
    float avg_voltage;
    int min_voltage_cell;
    int max_voltage_cell;
    float voltage_diff;
};

// 夹爪状态
struct GripperStatus {
    bool ready;
    bool position_reached;
    bool torque_reached;
    int alarm;
    int position;     // 添加：当前位置
};

// 回调函数类型
using AnalogCallback = std::function<void(int, float)>;
using DigitalCallback = std::function<void(int, int)>;

class PeripheralsClient {
public:
    PeripheralsClient();
    explicit PeripheralsClient(const std::string& server_url);
    ~PeripheralsClient();
    
    // 连接设置
    bool connect(const std::string& server_url);
    void disconnect();
    bool isConnected() const;
    
    // 系统接口
    bool healthCheck();
    bool getDeviceStatus(std::map<std::string, bool>& status);
    
    // 模拟量IO接口
    bool readAnalogVoltage(int channel, float& voltage);
    bool readAnalogCurrent(int channel, float& current);
    bool readAnalogRaw(int channel, int& raw_value);
    bool readAllAnalog(std::vector<AnalogInfo>& channels);
    bool setAnalogVoltage(int channel, float voltage);
    bool setAnalogRaw(int channel, int raw_value);
    
    // 数字IO接口
    bool readDigitalInput(int board, int port, int& value);
    bool readDigitalInput(int channel, int& value);
    bool readAllDigitalInputs(std::vector<DigitalInfo>& inputs);
    bool setDigitalOutput(int board, int port, int value);
    bool setDigitalOutput(int channel, int value);
    bool setSimulationMode(bool enable);
    
    // 云台接口
    bool movePTZ(uint8_t servo_id, float angle);
    bool getPTZAngle(uint8_t servo_id, float& angle);
    bool stopPTZ(uint8_t servo_id);
    bool stopAllPTZ();
    bool homePTZ();
    
    // 夹爪接口
    bool setGripperPosition(int position);
    bool getGripperPosition(int& position);
    bool enableGripper(bool enable);
    bool stopGripper();
    bool getGripperStatus(GripperStatus& status);
    bool clearGripperAlarm();
    
    // BMS接口
    bool readBMSBasicInfo(BMSBasicInfo& info);
    bool readBMSCellVoltages(BMSCellVoltageInfo& info);
    bool setChargeMOS(bool on);
    bool setDischargeMOS(bool on);
    bool getMOSStatus(bool& charge_on, bool& discharge_on);
    bool resetBMSCapacity();
    bool clearBMSProtection();
    
    // 光源接口
    bool setLightBrightness(int channel, int brightness);
    bool getLightBrightness(int channel, int& brightness);
    bool setLightOnOff(int channel, bool on);
    bool saveLightParams();
    
    // 旋转吸盘接口
    bool setSuckerAngle(int angle);
    bool getSuckerAngle(int& angle);
    bool setSuckerDuty(int duty);
    bool getSuckerDuty(int& duty);
    bool homeSucker();
    
    // 异步接口
    void startAnalogMonitor(int interval_ms, AnalogCallback callback);
    void stopAnalogMonitor();
    void startDigitalMonitor(int interval_ms, DigitalCallback callback);
    void stopDigitalMonitor();

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

} // namespace peripherals_remote

#endif // PERIPHERALS_REMOTE_CLIENT_H
