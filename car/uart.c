#include "uart.h"  // 包含本模块头文件，获得UART发送函数声明及ti_msp_dl_config.h中的外设定义


void UART0_SendChar(char ch)  // 通过UART0阻塞发送一个字符
{
#if UART_PRINT_ENABLE  // 串口打印总开关：为0时本函数直接返回，所有打印（含printf）都不输出
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)ch);  // 调用驱动库阻塞式发送：等待UART0发送寄存器空闲后写入数据
#else
    (void)ch;  // 打印关闭：丢弃字符，避免未使用参数警告
#endif  /* UART_PRINT_ENABLE结束 */
}

void UART0_SendString(const char *str)  // 通过UART0发送一个以'\0'结尾的字符串
{
    while (*str) {  // 逐字符遍历，直到遇到字符串结束符'\0'
        UART0_SendChar(*str++);  // 发送当前字符，然后指针后移到下一个字符
    }  /* while字符串遍历结束 */
}

int fputc(int ch, FILE *stream)  // 重定向标准库fputc到UART0，使printf可直接输出到串口
{
    UART0_SendChar((char)ch);  // 把要输出的字符经UART0发送出去（stream参数未使用）
    return ch;  // 按fputc约定返回写入的字符
}

void UART0_SendInt(int32_t num)  // 以十进制形式通过UART0发送一个32位有符号整数
{
    char buf[12];  // 字符缓冲区，32位整数最多10位数字加符号和结束符，12字节足够
    uint8_t i = 0;  // 缓冲区下标，记录已拆出的数字个数

    if (num == 0) {  // 特判0：下面的取余循环处理不了0
        UART0_SendChar('0');  // 直接发送字符'0'
        return;  // 处理完毕直接返回
    }  /* if(num==0)结束 */

    if (num < 0) {  // 负数先输出负号
        UART0_SendChar('-');  // 发送负号字符
        num = -num;  // 取绝对值，后续按正数处理
    }  /* if(num<0)结束 */

    while (num > 0) {  // 从低位到高位逐位拆出数字（逆序存入缓冲区）
        buf[i++] = num % 10 + '0';  // 取最低位数字转成ASCII字符存入缓冲区，下标加1
        num /= 10;  // 去掉已处理的最低位
    }  /* while拆位循环结束 */

    while (i > 0) UART0_SendChar(buf[--i]);  // 逆序发送缓冲区中的字符，得到正确的高位在前顺序
}

void UART0_SendHex8(uint8_t data)  // 以两位十六进制形式通过UART0发送一个字节（如 0x3F 发送"3F"）
{
    uint8_t hi = (data >> 4) & 0x0F;  // 取高4位（高半字节）
    uint8_t lo = data & 0x0F;  // 取低4位（低半字节）

    UART0_SendChar(hi < 10 ? '0' + hi : 'A' + hi - 10);  // 高半字节转成十六进制字符(0-9或A-F)并发送
    UART0_SendChar(lo < 10 ? '0' + lo : 'A' + lo - 10);  // 低半字节转成十六进制字符(0-9或A-F)并发送
}

void UART0_SendFloat2(float value)
{
    int32_t value_x100;

    if (value < 0.0f) {
        UART0_SendChar('-');
        value = -value;
    }

    value_x100 = (int32_t)(value * 100.0f + 0.5f);

    UART0_SendInt(value_x100 / 100);
    UART0_SendChar('.');

    if ((value_x100 % 100) < 10) {
        UART0_SendChar('0');
    }

    UART0_SendInt(value_x100 % 100);
}
