/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: nexus_manage/include/nexus_manage/components/command_calculator_component.hpp
 * @Description: 指令计算组件，负责根据遥操作输入计算末端执行器目标位姿，支持Scaling和Incremental两种模式
 * 
 * Copyright (c) 2025 by Pegasus, All Rights Reserved. 
 */

#ifndef __COMMAND_CALCULATOR_COMPONENT_HPP__
#define __COMMAND_CALCULATOR_COMPONENT_HPP__

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/logger.hpp>
#include <infra_msg/msg/end_pose_cmd.hpp>
#include <infra_msg/msg/robot_ee_cmd.hpp>
#include "nexus_manage/components/end_effector_config.hpp"
#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <vector>


/*******************************************************************************
 * Structure definition
 ******************************************************************************/
namespace nexus_manage {
namespace components {

/**
  * @brief  遥操作控制模式枚举
  * @param  null
  * @retval null
  * @usage  
  */
enum class TeleopMode {
    SCALING,      // Scaling模式：姿态偏差×pos_scaling + 初始姿态（初始姿态不变）
    INCREMENTAL   // 增量模式：姿态偏差×vel_scaling + 初始姿态（每周期更新初始姿态）
};

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  指令计算组件类
  * @param  null
  * @retval null
  * @usage  
  */
class CommandCalculatorComponent {
public:
    struct Impl;
    /**
      * @brief  构造函数
      * @param  logger  ROS2 logger
      * @retval null
      * @usage  
      */
    explicit CommandCalculatorComponent(rclcpp::Logger logger);
    ~CommandCalculatorComponent();
    
    /**
      * @brief  初始化末端执行器配置
      * @param  ee_configs  末端执行器配置列表
      * @retval null
      * @usage  
      */
    void initializeEndEffectorConfigs(const std::vector<EndEffectorConfig>& ee_configs);
    
    /**
      * @brief  设置 type="ee" 的位置缩放因子（线程安全，三轴独立）
      * @param  x  X轴位置缩放因子
      * @param  y  Y轴位置缩放因子
      * @param  z  Z轴位置缩放因子
      * @retval null
      * @usage
      */
    void setEEPosScalingFactor(double x, double y, double z);

    /**
      * @brief  设置 type="ee" 的速度缩放因子（线程安全，三轴独立）
      * @param  x  X轴速度缩放因子
      * @param  y  Y轴速度缩放因子
      * @param  z  Z轴速度缩放因子
      * @retval null
      * @usage
      */
    void setEEVelScalingFactor(double x, double y, double z);
    
    /**
      * @brief  设置 type="ee" 的旋转缩放因子（线程安全）
      * @param  scale  旋转缩放因子
      * @retval null
      * @usage  
      */
    void setEERotScalingFactor(double scale);
    
    /**
      * @brief  设置 type="ee" 的旋转RPY限幅参数（线程安全）
      * @param  limits  6维向量 [r_min, r_max, p_min, p_max, y_min, y_max] (rad)
      * @retval null
      * @usage  
      */
    void setEERpyLimits(const std::vector<double>& limits);

    /**
      * @brief  设置 type="ee" 的 base frame 绕Z轴旋转角度（线程安全）
      * @param  deg  旋转角度(度)，绕Z轴逆时针为正
      * @retval null
      */
    void setEEBaseFrameRotationZDeg(double deg);
    
    /**
      * @brief  设置 type="gripper" 的缩放因子（线程安全）
      * @param  scale  缩放因子
      * @retval null
      * @usage
      */
    void setGripperScalingFactor(double scale);

    /**
      * @brief  设置 gripper 值的符号翻转（线程安全）
      * @param  flip  true 时 gripper 输出值取反
      * @retval null
      * @usage
      */
    void setGripperSignFlip(bool flip);

    /**
      * @brief  获取 gripper 符号翻转状态（线程安全）
      * @param  null
      * @retval 是否翻转符号
      * @usage
      */
    bool getGripperSignFlip() const;

    /**
      * @brief  获取 type="ee" 的 X 轴位置缩放因子（线程安全）
      * @param  null
      * @retval X轴位置缩放因子
      * @usage
      */
    double getEEPosScalingFactorX() const;

    /**
      * @brief  获取 type="ee" 的 Y 轴位置缩放因子（线程安全）
      * @param  null
      * @retval Y轴位置缩放因子
      * @usage
      */
    double getEEPosScalingFactorY() const;

    /**
      * @brief  获取 type="ee" 的 Z 轴位置缩放因子（线程安全）
      * @param  null
      * @retval Z轴位置缩放因子
      * @usage
      */
    double getEEPosScalingFactorZ() const;

    /**
      * @brief  获取 type="ee" 的 X 轴速度缩放因子（线程安全）
      * @param  null
      * @retval X轴速度缩放因子
      * @usage
      */
    double getEEVelScalingFactorX() const;

    /**
      * @brief  获取 type="ee" 的 Y 轴速度缩放因子（线程安全）
      * @param  null
      * @retval Y轴速度缩放因子
      * @usage
      */
    double getEEVelScalingFactorY() const;

    /**
      * @brief  获取 type="ee" 的 Z 轴速度缩放因子（线程安全）
      * @param  null
      * @retval Z轴速度缩放因子
      * @usage
      */
    double getEEVelScalingFactorZ() const;
    
    /**
      * @brief  获取 type="ee" 的旋转缩放因子（线程安全）
      * @param  null
      * @retval 旋转缩放因子
      * @usage  
      */
    double getEERotScalingFactor() const;
    
    /**
      * @brief  获取 type="ee" 的旋转RPY限幅参数（线程安全）
      * @param  null
      * @retval RPY限幅向量
      * @usage  
      */
    Eigen::VectorXd getEERpyLimits() const;
    
    /**
      * @brief  获取 type="gripper" 的缩放因子（线程安全）
      * @param  null
      * @retval 缩放因子
      * @usage  
      */
    double getGripperScalingFactor() const;
    
    /**
      * @brief  设置遥操作控制模式（线程安全）
      * @param  mode  控制模式
      * @retval null
      * @usage  
      */
    void setTeleopMode(TeleopMode mode);
    
    /**
      * @brief  获取当前遥操作控制模式（线程安全）
      * @param  null
      * @retval 控制模式
      * @usage  
      */
    TeleopMode getTeleopMode() const;
    
    /**
      * @brief  进入遥操运行状态时更新初始姿态
      * @param  teleop_poses  主臂当前姿态
      * @param  robot_poses  从臂当前姿态
      * @retval null
      * @usage  
      */
    void updateInitialPoses(
        const std::map<std::string, infra_msg::msg::EndPoseCmd>& teleop_poses,
        const std::map<std::string, infra_msg::msg::EndPoseCmd>& robot_poses
    );
    
    /**
      * @brief  计算所有末端执行器的目标位姿指令
      * @param  teleop_current_poses  主臂当前姿态
      * @param  output_cmd  输出的机器人末端执行器指令
      * @retval 计算是否成功
      * @usage  
      */
    bool calculateCommands(
        const std::map<std::string, infra_msg::msg::EndPoseCmd>& teleop_current_poses,
        infra_msg::msg::RobotEECmd& output_cmd
    );

private:
    std::unique_ptr<Impl> impl_;
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace components
} // namespace nexus_manage

#endif
