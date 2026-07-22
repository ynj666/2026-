#ifndef OLED_H                        // 头文件保护：如果没有定义过OLED_H宏（防止重复包含）
#define OLED_H                        // 定义OLED_H宏，标记本头文件已被包含过

#include <stdint.h>                   // 包含标准整数类型头文件，提供uint8_t等类型定义

void WriteCmd(void);                  // 声明：发送SSD1306初始化命令序列（初始化时调用）
void OLED_WR_CMD(uint8_t cmd);        // 声明：向OLED写入一条命令字节
void OLED_WR_DATA(uint8_t data);      // 声明：向OLED写入一个显示数据字节

void OLED_Init(void);                 // 声明：OLED初始化（配置GPIO、发送初始化命令、清屏）
void OLED_Clear(void);                // 声明：清屏，把128x64显存全部清零
void OLED_Display_On(void);           // 声明：开启OLED显示
void OLED_Display_Off(void);          // 声明：关闭OLED显示（休眠省电）
void OLED_Set_Pos(uint8_t x, uint8_t y);  // 声明：设置显存写入位置，x为列(0~127)，y为页(0~7)
void OLED_On(void);                   // 声明：全屏点亮测试（显存全部写0x01）

void OLED_ShowRectangle(uint8_t x, uint8_t y, uint8_t high);  // 声明：在(x,y)处画高为high的实心竖条
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);  // 声明：显示单个字符，Char_Size为16或8
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size);  // 声明：显示字符串，自动换行
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2);  // 声明：显示无符号数字，len位，前导零显示空格
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int num, uint8_t len, uint8_t size2);  // 声明：显示带符号数字（带+/-号）
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);  // 声明：显示16x16点阵汉字，no为字库序号
void OLED_Draw12864BMP(uint8_t num);  // 声明：全屏显示128x64位图，num为图片编号（从1开始）
void OLED_ShowFloat2(uint8_t x, uint8_t y, float value, uint8_t int_len, uint8_t size);

#endif                                // 头文件保护结束（与开头的#ifndef OLED_H配对）
