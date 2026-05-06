/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: human_data/include/human_data/common/thread/rt_thread.hpp
 * @Description: 实时线程类，提供实时调度、CPU绑定、内存锁定等功能
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef __YJ_RT_THREAD_HPP__
#define __YJ_RT_THREAD_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <thread>
#include <functional>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <system_error>
#include <memory>
#include <chrono>
#include <atomic>
#include <iostream>
#include "nexus_log.hpp"
#include <cstdint>
#include <vector>
#include <mutex>
#include <condition_variable>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace yunji {
namespace robot {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  实时线程类
  * @param  null
  * @retval null
  */
class RealTimeThread {
public:
    /**
      * @brief  调度策略枚举
      * @param  null
      * @retval null
      */
    enum class SchedPolicy {
        FIFO = SCHED_FIFO,
        RR = SCHED_RR
    };

    /**
      * @brief  配置参数结构体
      * @param  null
      * @retval null
      */
    struct Config {
        SchedPolicy policy = SchedPolicy::FIFO;
        int priority = 80;               // 1-99 (越高越优先)
        int cpu_core = -1;               // -1 = 不绑定
        bool lock_memory = true;
        size_t memory_pool_size = 1024 * 1024; // 默认1MB内存池
        std::chrono::microseconds period{0}; // 执行周期，0表示非周期性
    };

    /**
      * @brief  内存池类
      * @param  null
      * @retval null
      */
    class MemoryPool {
    public:
        /**
          * @brief  构造函数
          * @param  size 内存池大小
          * @retval null
          */
        MemoryPool(size_t size) 
            : pool_(new uint8_t[size]), size_(size), offset_(0) {}
        
        /**
          * @brief  析构函数
          * @param  null
          * @retval null
          */
        ~MemoryPool() = default;
        
        /**
          * @brief  分配内存
          * @param  size 分配大小
          * @retval 内存指针
          */
        void* allocate(size_t size) {
            if (offset_ + size > size_) return nullptr;
            
            void* ptr = pool_.get() + offset_;
            offset_ += size;
            return ptr;
        }
        
        /**
          * @brief  重置内存池
          * @param  null
          * @retval null
          */
        void reset() {
            offset_ = 0;
        }
        
        /**
          * @brief  获取当前使用量
          * @param  null
          * @retval 当前使用量
          */
        size_t usage() const {
            return offset_;
        }
        
        /**
          * @brief  获取总容量
          * @param  null
          * @retval 总容量
          */
        size_t capacity() const {
            return size_;
        }

    private:
        std::unique_ptr<uint8_t[]> pool_;
        size_t size_;
        size_t offset_;
    };

    // 回调函数类型定义
    typedef std::function<void(MemoryPool* pool)> RealtimeCallback;
    
    /**
      * @brief  线程状态枚举
      * @param  null
      * @retval null
      */
    enum class ThreadState {
        CREATED,
        RUNNING,
        STOPPED,
        ERROR
    };

    /**
      * @brief  构造函数
      * @param  config 配置参数
      * @param  callback 回调函数
      * @retval null
      */
    RealTimeThread(const Config& config, RealtimeCallback callback) 
        : config_(config), callback_(callback), state_(ThreadState::CREATED) {
        
        // 创建内存池
        memory_pool_ = std::make_unique<MemoryPool>(config.memory_pool_size);
        
        // 创建线程
        thread_ = std::thread(&RealTimeThread::threadFunction, this);
    }

    /**
      * @brief  析构函数
      * @param  null
      * @retval null
      */
    ~RealTimeThread() {
        stop();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    /**
      * @brief  启动线程
      * @param  null
      * @retval null
      */
    void start() {
        if (state_ == ThreadState::CREATED) {
            {
                std::lock_guard<std::mutex> lock(start_mutex_);
                ready_to_start_ = true;
            }
            start_cv_.notify_one();
        }
    }

    /**
      * @brief  停止线程
      * @param  null
      * @retval null
      */
    void stop() {
        stop_requested_ = true;
    }
    
    /**
      * @brief  等待线程结束
      * @param  null
      * @retval null
      */
    void join() {
        if (thread_.joinable()) {
            thread_.join();
        }
    }
    
    /**
      * @brief  获取线程状态
      * @param  null
      * @retval 线程状态
      */
    ThreadState state() const {
        return state_;
    }
    
    /**
      * @brief  获取内存池使用情况
      * @param  null
      * @retval 内存使用量
      */
    size_t memoryUsage() const {
        return memory_pool_ ? memory_pool_->usage() : 0;
    }
    
    /**
      * @brief  全局初始化（进程级别）
      * @param  null
      * @retval null
      */
    static void globalInit() {
        lockProcessMemory();
    }
    
    /**
      * @brief  配置线程为实时线程并绑定到指定CPU核心
      * @param  priority 实时优先级 (1-99)
      * @param  cpu_core 要绑定的CPU核心编号 (-1表示不绑定)
      * @param  policy 调度策略 (默认为FIFO)
      * @retval 成功返回true，失败返回false
      */
    static bool configureRealtimeThread(int priority = 80, int cpu_core = -1, 
                                       SchedPolicy policy = SchedPolicy::FIFO) {
        // 设置调度策略和优先级
        sched_param param{};
        param.sched_priority = priority;
        
        if (pthread_setschedparam(pthread_self(), static_cast<int>(policy), &param) != 0) {
            NEXUS_CERR << "Failed to set thread scheduling policy" << std::endl;
            return false;
        }

        // 设置CPU亲和性
        if (cpu_core >= 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(cpu_core, &cpuset);
            
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
                NEXUS_CERR << "Failed to set CPU affinity" << std::endl;
                return false;
            }
        }

        return true;
    }
    
    /**
      * @brief  锁定当前线程的内存
      * @param  null
      * @retval 成功返回true，失败返回false
      */
    static bool lockCurrentThreadMemory() {
        if (mlockall(MCL_CURRENT) != 0) {
            NEXUS_CERR << "Failed to lock current thread memory" << std::endl;
            return false;
        }
        return true;
    }

private:
    /**
      * @brief  线程主函数
      * @param  null
      * @retval null
      */
    void threadFunction() {
        // 等待start信号
        {
            std::unique_lock<std::mutex> lock(start_mutex_);
            start_cv_.wait(lock, [this] { return ready_to_start_; });
        }

        try {
            // 应用实时配置
            applyRealtimeConfig();
            
            state_ = ThreadState::RUNNING;

            using Clock = std::chrono::steady_clock;

            while (!stop_requested_) {
            
                // 非周期性任务
                if (config_.period.count() == 0) {
                    // 重置内存池
                    memory_pool_->reset();

                    // 执行用户回调函数
                    callback_(memory_pool_.get());
                }
                // 周期性任务
                else {          
                    // 记录周期开始时间
                    auto cycle_start = Clock::now();
                    
                    // 重置内存池
                    memory_pool_->reset();
                    
                    // 执行用户回调函数
                    callback_(memory_pool_.get());
                    
                    // 计算实际执行时间
                    auto execution_time = Clock::now() - cycle_start;
                    
                    // 检查是否超时
                    if (execution_time > config_.period) {
                        handleError("Cycle time exceeded! Required: " + 
                                std::to_string(config_.period.count()) + "us, Actual: " +
                                std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(execution_time).count()) + "us");
                    }
                    
                    // 计算剩余时间并精确休眠
                    auto remaining_time = config_.period - execution_time;
                    
                    if (remaining_time.count() > 0) {
                        // 使用单调时钟精确休眠
                        std::this_thread::sleep_for(remaining_time);
                    }
                }
            }
            
            state_ = ThreadState::STOPPED;
        }
        catch (const std::exception& e) {
            state_ = ThreadState::ERROR;
            handleError(e.what());
        }
    }

    /**
      * @brief  应用实时配置
      * @param  null
      * @retval null
      */
    void applyRealtimeConfig() {
        // 1. 设置调度策略和优先级
        sched_param param{};
        param.sched_priority = config_.priority;
        
        if (pthread_setschedparam(
            pthread_self(), 
            static_cast<int>(config_.policy), 
            &param
        ) != 0) {
            throw std::system_error(errno, std::system_category(), 
                                  "Failed to set scheduling policy");
        }

        // 2. 设置CPU亲和性
        if (config_.cpu_core >= 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(config_.cpu_core, &cpuset);
            
            if (pthread_setaffinity_np(
                pthread_self(), 
                sizeof(cpu_set_t), 
                &cpuset
            ) != 0) {
                throw std::system_error(errno, std::system_category(), 
                                      "Failed to set CPU affinity");
            }
        }

        // 3. 内存锁定
        if (config_.lock_memory) {
            lockThreadMemory();
        }
    }

    /**
      * @brief  锁定线程内存
      * @param  null
      * @retval null
      */
    void lockThreadMemory() {
        // 锁定当前线程栈
        if (mlockall(MCL_CURRENT) != 0) {
            throw std::system_error(errno, std::system_category(), 
                                  "Failed to lock thread memory");
        }
    }

    /**
      * @brief  锁定进程内存
      * @param  null
      * @retval null
      */
    static void lockProcessMemory() {
        // 锁定进程所有内存
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            throw std::system_error(errno, std::system_category(), 
                                  "Failed to lock process memory");
        }
    }
    
    /**
      * @brief  处理错误
      * @param  message 错误消息
      * @retval null
      */
    void handleError(const std::string& message) {
        NEXUS_CERR << "RealTimeThread ERROR: " << message << std::endl;
    }

    Config config_;
    RealtimeCallback callback_;
    std::unique_ptr<MemoryPool> memory_pool_;
    std::thread thread_;
    std::atomic<ThreadState> state_;
    std::atomic<bool> stop_requested_{false};

    std::mutex start_mutex_;
    std::condition_variable start_cv_;
    bool ready_to_start_ = false;
};

/**
  * @brief  实时安全内存分配器模板类
  * @param  T 元素类型
  * @retval null
  */
template <typename T>
class RealtimeAllocator {
public:
    using value_type = T;
    
    /**
      * @brief  默认构造函数
      * @param  null
      * @retval null
      */
    RealtimeAllocator() = default;
    
    /**
      * @brief  从内存池构造
      * @param  pool 内存池指针
      * @retval null
      */
    explicit RealtimeAllocator(RealTimeThread::MemoryPool* pool) 
        : pool_(pool) {}
    
    template <typename U>
    RealtimeAllocator(const RealtimeAllocator<U>& other) noexcept 
        : pool_(other.pool_) {}
    
    /**
      * @brief  分配内存
      * @param  n 元素数量
      * @retval 内存指针
      */
    T* allocate(size_t n) {
        if (!pool_) throw std::bad_alloc();
        
        void* ptr = pool_->allocate(n * sizeof(T));
        if (!ptr) throw std::bad_alloc();
        
        return static_cast<T*>(ptr);
    }
    
    /**
      * @brief  释放内存（内存池分配器通常不释放内存）
      * @param  p 内存指针
      * @param  n 元素数量
      * @retval null
      */
    void deallocate(T* p, size_t n) noexcept {
        // 内存池分配器通常不释放内存
        // 内存会在每个周期开始时重置
    }
    
    /**
      * @brief  获取内存池指针
      * @param  null
      * @retval 内存池指针
      */
    RealTimeThread::MemoryPool* pool() const {
        return pool_;
    }

private:
    RealTimeThread::MemoryPool* pool_ = nullptr;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace robot
} // namespace yunji

#endif // __YJ_RT_THREAD_HPP__
