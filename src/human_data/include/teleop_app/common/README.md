# Teleop App Common Libraries

本目录包含从 yunji_sdk 引入的公共库，供 human_data 包内部使用。

## 目录结构

```
common/
├── thread/                         # 线程相关工具
│   ├── circular_queue.hpp         # 循环队列（线程安全消息队列）
│   └── rt_thread.hpp              # 实时线程封装
│
└── fsm/                           # 状态机框架
    ├── state_interface.hpp        # 状态接口定义
    ├── state_machine.hpp          # 状态机接口定义
    └── state_machine_engine.cpp   # 状态机引擎实现（位于 src/common/fsm/）
```

## 组件说明

### 线程工具 (thread/)

#### CircularQueue<T>
线程安全的循环消息队列模板类。

**特性**：
- 模板类，支持任意类型的消息
- 队列满时自动丢弃最老的元素
- 支持超时等待的 pop 操作
- 使用互斥锁和条件变量实现线程安全

**用法示例**：
```cpp
#include "human_data/common/thread/circular_queue.hpp"

// 创建最大容量为 10 的消息队列
CircularQueue<MyMessage> queue(10);

// 生产者：推送消息
auto msg = std::make_shared<MyMessage>();
queue.push(msg);

// 消费者：获取消息（最多等待 100ms）
auto received = queue.pop(std::chrono::milliseconds(100));
if (received) {
    // 处理消息
}
```

#### RealTimeThread
实时线程封装类，提供实时性保证。

**特性**：
- 支持 FIFO/RR 调度策略
- 可配置优先级（1-99）
- CPU 核心绑定
- 内存锁定（避免页面交换）
- 实时安全的内存池管理
- 支持周期性和非周期性任务

**用法示例**：
```cpp
#include "human_data/common/thread/rt_thread.hpp"

using namespace yunji::robot;

// 配置实时线程参数
RealTimeThread::Config config;
config.policy = RealTimeThread::SchedPolicy::FIFO;
config.priority = 80;
config.cpu_core = 2;  // 绑定到 CPU 核心 2
config.lock_memory = true;
config.period = std::chrono::microseconds(1000);  // 1ms 周期

// 创建实时线程
auto rt_thread = std::make_unique<RealTimeThread>(
    config,
    [](RealTimeThread::MemoryPool* pool) {
        // 实时任务回调函数
        // 使用 pool 进行实时安全的内存分配
    }
);

// 启动线程
rt_thread->start();

// 停止线程
rt_thread->stop();
rt_thread->join();
```

### 状态机框架 (fsm/)

#### IState
状态接口抽象类。

**接口**：
- `name()`: 返回状态名称
- `on_entry()`: 进入状态时调用
- `on_exit()`: 退出状态时调用
- `on_update()`: 状态更新时调用
- `handle_event()`: 处理事件，返回目标状态名

**用法示例**：
```cpp
#include "teleop_app/common/fsm/state_interface.hpp"

class MyState : public robot_sdk::IState {
public:
    std::string name() const override { return "MyState"; }
    
    void on_entry() override {
        // 进入状态的初始化逻辑
    }
    
    void on_exit() override {
        // 退出状态的清理逻辑
    }
    
    void on_update() override {
        // 状态的周期性更新逻辑
    }
    
    std::string handle_event(const std::string& event) override {
        if (event == "go_to_next") {
            return "NextState";  // 转换到 NextState
        }
        return "";  // 不转换
    }
};
```

#### StateMachine
状态机接口和实现。

**接口**：
- `register_state()`: 注册状态
- `initialize()`: 初始化状态机到指定初始状态
- `process_event()`: 处理事件，可能触发状态转换
- `update()`: 更新当前状态
- `current_state()`: 获取当前状态名称

**用法示例**：
```cpp
#include "human_data/common/fsm/state_machine.hpp"
#include "human_data/common/fsm/state_interface.hpp"

// 创建状态机
auto fsm = robot_sdk::create_state_machine();

// 创建并注册状态
auto state1 = std::make_shared<MyState1>();
auto state2 = std::make_shared<MyState2>();
fsm->register_state("State1", state1);
fsm->register_state("State2", state2);

// 初始化到 State1
fsm->initialize("State1");

// 事件处理
fsm->process_event("some_event");

// 定期更新
fsm->update();

// 查询当前状态
std::string current = fsm->current_state();
```

## 线程安全性

- **CircularQueue**: 完全线程安全，可在多个生产者和消费者之间使用
- **RealTimeThread**: 线程状态查询是原子操作，启动/停止操作线程安全
- **StateMachine**: 所有状态转换和事件处理都由互斥锁保护，完全线程安全

## 注意事项

1. **实时线程**：使用实时线程需要 root 权限或配置适当的系统限制
2. **内存锁定**：内存锁定会占用物理内存，确保系统有足够的 RAM
3. **状态机**：`on_entry`、`on_exit`、`on_update` 可能耗时，状态机引擎已优化避免长时间持锁

## 源代码来源

这些库来自 yunji_sdk 项目：
- `yunji_sdk/include/yunji/thread/` → `human_data/include/human_data/common/thread/`
- `yunji_sdk/include/yunji/robot/fsm/` → `human_data/include/human_data/common/fsm/`
- `yunji_sdk/src/yunji/robot/fsm/` → `human_data/src/common/fsm/`
