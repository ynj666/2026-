#ifndef BMI270_SPI_H              // 头文件保护：防止本头文件被重复包含
#define BMI270_SPI_H              // 定义保护宏，配合上面的#ifndef使用

#include <stdint.h>               // 提供uint8_t、uint32_t等标准整型定义
#include "bmi2_defs.h"            // BMI270官方驱动库的类型定义（如BMI2_INTF_RETURN_TYPE）

#define BMI270_SPI_CHIP_ID_VALUE  0x24    // BMI270芯片ID的期望值，用于校验芯片是否正常连接

void BMI270_SPI_Init(void);             // SPI接口初始化：片选拉高并延时等待BMI270上电稳定
uint8_t BMI270_SPI_ReadID(void);        // 读取BMI270芯片ID，正常应返回0x24

BMI2_INTF_RETURN_TYPE BMI270_SPI_Read(uint8_t reg_addr, uint8_t *reg_data,
                                      uint32_t len, void *intf_ptr);    // 驱动库读回调：从reg_addr连续读len字节到reg_data
BMI2_INTF_RETURN_TYPE BMI270_SPI_Write(uint8_t reg_addr, const uint8_t *reg_data,
                                       uint32_t len, void *intf_ptr);   // 驱动库写回调：向reg_addr连续写入len字节
void BMI270_SPI_DelayUs(uint32_t period, void *intf_ptr);               // 微秒级延时回调，供BMI270驱动库内部调用

#endif                          // 头文件保护结束（对应开头的#ifndef）