/*
 * @Author: Infra Embedded
 * @Date: 2024-09-21 09:41:14
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: human_data/include/human_data/common/thread/circular_queue.hpp
 * @Description: 循环队列模板类，线程安全的消息队列
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef CIRCCULAR_QUEUE_HPP
#define CIRCCULAR_QUEUE_HPP

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <iostream>
#include <chrono>

/*******************************************************************************
 * Macro definition
 ******************************************************************************/

/*******************************************************************************
 * Structure definition
 ******************************************************************************/

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  通用的消息队列类模板
  * @param  T 队列元素类型
  * @retval null
  */
template <typename T>
class CircularQueue {
public:
    /**
      * @brief  构造函数，初始化最大容量
      * @param  maxSize 最大容量
      * @retval null
      */
    CircularQueue(size_t maxSize = 10) : maxSize_(maxSize) {}

    /**
      * @brief  推送数据到队列
      * @param  data 数据指针
      * @retval null
      */
    void push(std::shared_ptr<T> data) {
        std::unique_lock<std::mutex> lock(mutex_);

        // 队列已满时丢弃最老的元素
        if (queue_.size() >= maxSize_) {
            queue_.pop();  // 丢弃最老的元素
        }

        // 将新消息推送到队列
        queue_.push(data);

        // 唤醒消费者线程
        cvEmpty_.notify_one();
    }

    /**
      * @brief  从队列中获取数据，支持最大阻塞时间
      * @param  timeout 超时时间（毫秒）
      * @retval 数据指针，超时返回nullptr
      */
    std::shared_ptr<T> pop(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);

        // 阻塞直到队列不为空或超时
        if (!cvEmpty_.wait_for(lock, timeout, [this]() { return !queue_.empty(); })) {
            // 超时返回空指针
            return nullptr;
        }

        auto message = queue_.front();
        queue_.pop();
        
        return message;
    }

private:
    std::queue<std::shared_ptr<T>> queue_;  // 使用 std::queue 来存储数据
    size_t maxSize_;  // 队列最大容量
    std::mutex mutex_;
    std::condition_variable cvEmpty_; // 用于队列不为空时通知消费者
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

#endif // CIRCCULAR_QUEUE_HPP
