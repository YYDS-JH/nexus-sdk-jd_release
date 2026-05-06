/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/nodes/teleop_arm_driver_node.hpp
 * @Description: Teleop Arm Driver Node，负责管理多个机械臂适配器，实现ROS2与硬件之间的通信桥梁
 *
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved.
 */

#ifndef TELEOP_ADAPTER_TELEOP_ARM_DRIVER_NODE_HPP
#define TELEOP_ADAPTER_TELEOP_ARM_DRIVER_NODE_HPP

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <memory>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
 * @brief  遥操作机械臂驱动节点类
 * @param  null
 * @retval null
 */
class TeleopArmDriverNode : public rclcpp::Node {
public:
    /**
     * @brief  构造函数
     * @param  options ROS2节点选项
     * @retval null
     */
    explicit TeleopArmDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief  析构函数
     * @param  null
     * @retval null
     */
    ~TeleopArmDriverNode();

    /**
     * @brief  初始化节点
     * @param  null
     * @retval 是否成功
     */
    bool initialize();

    /**
     * @brief  关闭节点
     * @param  null
     * @retval null
     */
    void shutdown();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_TELEOP_ARM_DRIVER_NODE_HPP
