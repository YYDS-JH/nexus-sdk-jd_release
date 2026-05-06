/*
 * @Author: Infra Embedded
 * @Date: 2024-12-22 18:34:58
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/include/teleop_adapter/utils/endian_utils.hpp
 * @Description: 字节序检测和转换工具
 * 
 * Copyright (c) 2024 by Infra Embedded, All Rights Reserved. 
 */

#ifndef TELEOP_ADAPTER_ENDIAN_UTILS_HPP
#define TELEOP_ADAPTER_ENDIAN_UTILS_HPP

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <cstdint>
#include <cstring>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Class definition
 ******************************************************************************/

/**
  * @brief  字节序工具类
  * @param  null
  * @retval null
  */
class EndianUtils {
public:
    /**
      * @brief  检测当前系统是否为小端序
      * @param  null
      * @retval true 为小端序，false 为大端序
      */
    static bool isLittleEndian() {
        union {
            uint32_t i;
            uint8_t c[4];
        } test = {0x01020304};
        
        return test.c[0] == 0x04;  // 小端：低地址存低字节
    }
    
    /**
      * @brief  16位整数字节序转换
      * @param  value 输入值
      * @retval 转换后的值
      */
    static uint16_t swap16(uint16_t value) {
        return ((value & 0xFF00) >> 8) |
               ((value & 0x00FF) << 8);
    }
    
    /**
      * @brief  32位整数字节序转换
      * @param  value 输入值
      * @retval 转换后的值
      */
    static uint32_t swap32(uint32_t value) {
        return ((value & 0xFF000000) >> 24) |
               ((value & 0x00FF0000) >> 8)  |
               ((value & 0x0000FF00) << 8)  |
               ((value & 0x000000FF) << 24);
    }
    
    /**
      * @brief  主机字节序转小端序（16位）
      * @param  value 主机字节序的值
      * @retval 小端序的值
      */
    static uint16_t hostToLittle16(uint16_t value) {
        if (isLittleEndian()) {
            return value;  // 已经是小端，无需转换
        } else {
            return swap16(value);  // 大端转小端
        }
    }
    
    /**
      * @brief  小端序转主机字节序（16位）
      * @param  value 小端序的值
      * @retval 主机字节序的值
      */
    static uint16_t littleToHost16(uint16_t value) {
        if (isLittleEndian()) {
            return value;  // 已经是小端，无需转换
        } else {
            return swap16(value);  // 小端转大端
        }
    }
    
    /**
      * @brief  主机字节序转小端序（32位）
      * @param  value 主机字节序的值
      * @retval 小端序的值
      */
    static uint32_t hostToLittle32(uint32_t value) {
        if (isLittleEndian()) {
            return value;  // 已经是小端，无需转换
        } else {
            return swap32(value);  // 大端转小端
        }
    }
    
    /**
      * @brief  小端序转主机字节序（32位）
      * @param  value 小端序的值
      * @retval 主机字节序的值
      */
    static uint32_t littleToHost32(uint32_t value) {
        if (isLittleEndian()) {
            return value;  // 已经是小端，无需转换
        } else {
            return swap32(value);  // 小端转大端
        }
    }
    
    /**
      * @brief  从字节数组读取16位小端序整数
      * @param  data 字节数组指针
      * @retval 主机字节序的值
      */
    static uint16_t readLittle16(const uint8_t* data) {
        uint16_t value = (static_cast<uint16_t>(data[1]) << 8) |
                         static_cast<uint16_t>(data[0]);
        return value;  // 已按小端序读取
    }
    
    /**
      * @brief  将16位整数写入字节数组（小端序）
      * @param  data 字节数组指针
      * @param  value 主机字节序的值
      * @retval null
      */
    static void writeLittle16(uint8_t* data, uint16_t value) {
        data[0] = static_cast<uint8_t>(value & 0xFF);
        data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }
    
    /**
      * @brief  从字节数组读取32位小端序整数
      * @param  data 字节数组指针
      * @retval 主机字节序的值
      */
    static uint32_t readLittle32(const uint8_t* data) {
        uint32_t value = (static_cast<uint32_t>(data[3]) << 24) |
                         (static_cast<uint32_t>(data[2]) << 16) |
                         (static_cast<uint32_t>(data[1]) << 8)  |
                         static_cast<uint32_t>(data[0]);
        return value;  // 已按小端序读取
    }
    
    /**
      * @brief  将32位整数写入字节数组（小端序）
      * @param  data 字节数组指针
      * @param  value 主机字节序的值
      * @retval null
      */
    static void writeLittle32(uint8_t* data, uint32_t value) {
        data[0] = static_cast<uint8_t>(value & 0xFF);
        data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
        data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    }
};

/*******************************************************************************
 * Function extern
 ******************************************************************************/

} // namespace teleop_adapter

#endif // TELEOP_ADAPTER_ENDIAN_UTILS_HPP
