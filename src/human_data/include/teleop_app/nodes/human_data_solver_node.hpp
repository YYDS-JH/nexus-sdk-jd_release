/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: human_data/include/teleop_app/nodes/human_data_solver_node.hpp
 * @Description: 人体数据解算节点，管理多个末端解算器和夹爪解算器
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef TELEOP_APP__NODES__HUMAN_DATA_SOLVER_NODE_HPP_
#define TELEOP_APP__NODES__HUMAN_DATA_SOLVER_NODE_HPP_

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <memory>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace human_data {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  人体数据解算节点
  * @param  null
  * @retval null
  * @note   架构：
  *         - 管理多个末端解算器和夹爪解算器
  *         - 每个解算器有独立的解算线程（在节点内部管理）
  *         - 定频发布线程收集所有解算器的结果并发布
  */
class HumanDataSolverNode : public rclcpp::Node {
public:
    /**
      * @brief  构造函数
      * @param  null
      * @retval null
      * @usage
      */
    HumanDataSolverNode();

    /**
      * @brief  析构函数
      * @param  null
      * @retval null
      * @usage
      */
    ~HumanDataSolverNode();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace human_data

#endif // TELEOP_APP__NODES__HUMAN_DATA_SOLVER_NODE_HPP_
