#include "gray.h"  // 包含本模块头文件，获得灰度传感器编号枚举和读取函数声明
#include "ti_msp_dl_config.h"  // 包含SysConfig生成的配置头文件，提供GPIOA/GPIOB、DL_GPIO_PIN_x等定义

typedef struct {  // 定义一路灰度传感器的引脚配置结构体
    GPIO_Regs *port;  // 该路传感器连接的GPIO端口寄存器基地址（如GPIOA、GPIOB）
    uint32_t pin;  // 该路传感器连接的引脚掩码（如DL_GPIO_PIN_25）
} GRAY_Config;  /* 结构体类型GRAY_Config定义结束 */

/* 换灰度引脚只改这里 */  // 硬件改线时只需修改下表中对应的端口和引脚
static const GRAY_Config gray_config[GRAY_NUM] = {  // 8路灰度传感器的引脚映射表，下标与GRAY_ID枚举一一对应
    {GPIOA, DL_GPIO_PIN_25},   // GRAY_1：接PA25
    {GPIOA, DL_GPIO_PIN_26},   // GRAY_2：接PA26
    {GPIOA, DL_GPIO_PIN_27},   // GRAY_3：接PA27
    {GPIOB, DL_GPIO_PIN_24},   // GRAY_4：接PB24
    {GPIOB, DL_GPIO_PIN_19},   // GRAY_5：接PB19
    {GPIOA, DL_GPIO_PIN_22},   // GRAY_6：接PA22
    {GPIOB, DL_GPIO_PIN_18},   // GRAY_7：接PB18
    {GPIOA, DL_GPIO_PIN_24},   // GRAY_8：接PA24
};  /* 引脚映射表定义结束 */

/* 读取单路原始电平：高=1，低=0 */  // 返回指定一路灰度传感器的数字电平
uint8_t GRAY_ReadBit(GRAY_ID id)  // 读取编号为id的那一路灰度传感器，返回0或1
{
    if (id >= GRAY_NUM) {  // 参数合法性检查：编号超出范围
        return 0;  // 非法编号直接返回0，防止访问映射表越界
    }  /* if参数检查结束 */

    return (DL_GPIO_readPins(gray_config[id].port, gray_config[id].pin) != 0) ? 1 : 0;  // 读取对应引脚电平，非0即高电平返回1，否则返回0
}

/* bit0=GRAY1, bit1=GRAY2 ... bit7=GRAY8 */  // 返回值每一位对应一路传感器的电平
uint8_t GRAY_ReadAll(void)  // 一次读取全部8路灰度传感器，打包成一个字节返回
{
    uint8_t value = 0;  // 结果字节，初始为0（所有位清零）
    uint8_t i;  // 循环变量，遍历8路传感器

    for (i = 0; i < GRAY_NUM; i++) {  // 依次读取第1路到第8路
        if (GRAY_ReadBit((GRAY_ID)i)) {  // 若第i路读到高电平
            value |= (1 << i);  // 把结果字节的第i位置1
        }  /* if高电平判断结束 */
    }  /* for循环结束 */

    return value;  // 返回打包好的8路电平结果
}