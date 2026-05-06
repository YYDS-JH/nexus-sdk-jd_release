/*
 * @Description: Shared end-effector configuration type (used by config parser and command calculator)
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

#ifndef __NEXUS_MANAGE_END_EFFECTOR_CONFIG_HPP__
#define __NEXUS_MANAGE_END_EFFECTOR_CONFIG_HPP__

#include <string>
#include <vector>

namespace nexus_manage {
namespace components {

/**
 * @brief  末端执行器配置结构体
 */
struct EndEffectorConfig {
    std::string name;                       // 末端执行器名称
    std::string type;                       // 类型："ee"或"gripper"
    std::vector<double> teleop_pos_values;  // 主臂位置初始值
    std::vector<double> robot_pos_values;   // 从臂位置初始值
};

} // namespace components
} // namespace nexus_manage

#endif
