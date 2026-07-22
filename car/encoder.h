#ifndef __ENCODER_H__         // 头文件保护：防止重复包含（若未定义__ENCODER_H__则继续）
#define __ENCODER_H__         // 定义宏__ENCODER_H__，标记本头文件已被包含

#include <stdint.h>           // 引入标准整数类型定义（uint8_t、int32_t等）

typedef enum {                // 定义编码器编号枚举
    ENCODER_1 = 0,            // 编码器1，编号0（左轮）
    ENCODER_2,                // 编码器2，编号1（右轮）
    ENCODER_3,                // 编码器3，编号2（电机C对应的轮子）
    ENCODER_4,                // 编码器4，编号3（电机D对应的轮子）
    ENCODER_NUM               // 编码器总数，值为4（用于数组大小和越界判断）
} ENCODER_ID;                 // 枚举类型命名为ENCODER_ID

void Encoder_Init(void);              // 编码器初始化：清零计数并使能GPIOB中断
int32_t Encoder_Get(ENCODER_ID id);   // 读取指定编码器的累计脉冲数
void Encoder_Clear(ENCODER_ID id);    // 清零指定编码器的脉冲计数
void Encoder_ClearAll(void);          // 清零所有编码器的脉冲计数

#endif                        // 头文件保护结束（与开头的#ifndef配对）
