#pragma once
#include "state_interface.hpp"
#include <memory>
#include <unordered_map>

namespace robot_sdk {

class StateMachine {
public:
    virtual ~StateMachine() = default;
    
    // 初始化状态机
    virtual void initialize(const std::string& initial_state) = 0;
    
    // 处理事件
    virtual void process_event(const std::string& event) = 0;
    
    // 状态更新
    virtual void update() = 0;
    
    // 获取当前状态
    virtual std::string current_state() const = 0;
    
    // 注册状态
    virtual void register_state(const std::string& name, StatePtr state) = 0;
};

// 创建状态机实例
std::unique_ptr<StateMachine> create_state_machine();

} // namespace robot_sdk
