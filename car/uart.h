#ifndef __UART_H__  // 头文件保护：防止本头文件被重复包含（第一次包含时宏未定义才继续）
#define __UART_H__  // 定义保护宏，再次包含时上面的#ifndef会失败从而跳过全部内容

#include <stdio.h>  // 包含标准输入输出头文件，提供FILE类型（fputc重定向用到）
#include "ti_msp_dl_config.h"  // 包含SysConfig生成的驱动配置头文件，提供UART_0_INST等外设宏和驱动库声明
#include <stdio.h>  // 重复包含stdio.h（原代码如此，有头文件保护不会出问题）

/* 串口打印总开关：1=打开打印，0=关闭打印。
   关闭后所有UART0输出（含printf）都不再发送，主循环也不会阻塞在串口发送上 */
#define UART_PRINT_ENABLE  1  // 改这里控制串口打印开关

void UART0_SendChar(char ch);  // 声明：通过UART0发送单个字符
void UART0_SendString(const char *str);  // 声明：通过UART0发送字符串
void UART0_SendInt(int32_t num);  // 声明：以十进制形式发送32位有符号整数
void UART0_SendHex8(uint8_t data);  // 声明：以两位十六进制形式发送一个字节
void UART0_SendFloat2(float value);

#endif  // 头文件保护结束，对应开头的#ifndef __UART_H__