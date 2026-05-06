#pragma once
#include <memory>
#include <string>

namespace robot_sdk {

class IState {
public:
    virtual ~IState() = default;
    
    // 状态名称
    virtual std::string name() const = 0;
    
    // 进入状态
    virtual void on_entry() {}
    
    // 退出状态
    virtual void on_exit() {}
    
    // 状态更新
    virtual void on_update() {}
    
    // 事件处理（返回目标状态名，空字符串表示不转换）
    virtual std::string handle_event(const std::string& event) = 0;
};

using StatePtr = std::shared_ptr<IState>;

} // namespace robot_sdk
