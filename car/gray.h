#ifndef __GRAY_H__  // 头文件保护：防止本头文件被重复包含
#define __GRAY_H__  // 定义保护宏，再次包含时跳过全部内容

#include <stdint.h>  // 包含标准整数类型头文件，提供uint8_t等类型

typedef enum {  // 定义8路灰度传感器的编号枚举
    GRAY_1 = 0,  // 第1路灰度传感器，编号0
    GRAY_2,  // 第2路，编号1
    GRAY_3,  // 第3路，编号2
    GRAY_4,  // 第4路，编号3
    GRAY_5,  // 第5路，编号4
    GRAY_6,  // 第6路，编号5
    GRAY_7,  // 第7路，编号6
    GRAY_8,  // 第8路，编号7
    GRAY_NUM  // 传感器总数（值为8），也用于参数合法性检查
} GRAY_ID;  /* 枚举类型GRAY_ID定义结束 */

uint8_t GRAY_ReadBit(GRAY_ID id);  // 声明：读取指定一路灰度传感器电平，返回0或1
uint8_t GRAY_ReadAll(void);  // 声明：一次读取全部8路电平，bit0~bit7分别对应GRAY_1~GRAY_8

#endif  // 头文件保护结束，对应开头的#ifndef __GRAY_H__