#include "oled.h"                    // 包含OLED驱动自身的头文件，拿到本模块的函数声明
#include "OLED_Font.h"               // 包含字库头文件，提供ASCII字符和汉字、图片的点阵字模数据
#include "ti_msp_dl_config.h"        // 包含SysConfig生成的芯片配置头文件，提供GPIO定义、delay_cycles、CPUCLK_FREQ等

/* ===== 引脚配置区：换引脚只改这里 ===== */
#define OLED_SCL_PORT      GPIOB          // OLED时钟线SCL所在的GPIO端口：GPIOB
#define OLED_SCL_PIN       DL_GPIO_PIN_2  // OLED时钟线SCL对应的引脚：PB2

#define OLED_SDA_PORT      GPIOB          // OLED数据线SDA所在的GPIO端口：GPIOB
#define OLED_SDA_PIN       DL_GPIO_PIN_3  // OLED数据线SDA对应的引脚：PB3

#define OLED_ADDR          0x3C           // OLED(SSD1306)的I2C从机地址：0x3C（7位地址）
/* ==================================== */

static const uint8_t OLED_INIT_CMD[] = {  // SSD1306初始化命令序列表，上电后按顺序逐条发送
    0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6,  // 关显示/列地址低/列地址高/起始行/页地址/对比度/段重映射/正常显示
    0xA8, 0x3F, 0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1,  // 多路复用率64/COM扫描方向/显示偏移0/时钟分频/预充电周期
    0xDA, 0x12, 0xDB, 0x40, 0x8D, 0x14, 0xAF               // COM硬件配置/VCOMH电平/使能电荷泵/开显示
};

static void OLED_Delay(void)              // 软件I2C用的短延时函数，保证总线时序满足要求
{
    delay_cycles(CPUCLK_FREQ / 1000000U); // 延时约1微秒：SSD1306支持400kHz，每比特约2~3us即可，比原来10us快约10倍
}

static void OLED_SCL(uint8_t level)       // 设置SCL时钟线电平，level非0输出高、为0输出低
{
    if (level) DL_GPIO_setPins(OLED_SCL_PORT, OLED_SCL_PIN);    // SCL引脚输出高电平
    else       DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN);  // SCL引脚输出低电平
}

static void OLED_SDA_Low(void)            // 将SDA数据线拉低（主动驱动为低电平）
{
    DL_GPIO_enableOutput(OLED_SDA_PORT, OLED_SDA_PIN);   // 使能SDA引脚的输出功能（推挽驱动）
    DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN);      // SDA引脚输出低电平
}

static void OLED_SDA_High(void)           // 释放SDA数据线（靠上拉电阻拉到高电平，模拟开漏）
{
    DL_GPIO_setPins(OLED_SDA_PORT, OLED_SDA_PIN);        // 输出寄存器置高（为再次使能输出做准备）
    DL_GPIO_disableOutput(OLED_SDA_PORT, OLED_SDA_PIN);  // 关闭输出驱动，引脚由内部上拉拉为高电平
}

static void OLED_I2C_Start(void)          // 产生I2C起始信号：SCL高电平期间SDA由高变低
{
    OLED_SDA_High();                      // 先释放SDA为高电平
    OLED_SCL(1);                          // SCL拉高
    OLED_Delay();                         // 延时保持时序
    OLED_SDA_Low();                       // SCL高电平期间拉低SDA，形成起始条件
    OLED_Delay();                         // 延时保持时序
    OLED_SCL(0);                          // 拉低SCL，准备发送数据位
}

static void OLED_I2C_Stop(void)           // 产生I2C停止信号：SCL高电平期间SDA由低变高
{
    OLED_SDA_Low();                       // 先拉低SDA
    OLED_SCL(1);                          // SCL拉高
    OLED_Delay();                         // 延时保持时序
    OLED_SDA_High();                      // SCL高电平期间释放SDA变高，形成停止条件
    OLED_Delay();                         // 延时保持时序
}

static void OLED_I2C_WriteByte(uint8_t data)  // 通过软件模拟I2C向OLED写入一个字节（MSB先发）
{
    uint8_t i;                            // 循环计数器，用于逐位发送

    for (i = 0; i < 8; i++) {             // 循环8次，每次发送1位（从最高位开始）
        if (data & 0x80) OLED_SDA_High(); // 当前最高位为1，SDA输出高电平
        else             OLED_SDA_Low();  // 当前最高位为0，SDA输出低电平

        OLED_Delay();                     // 等待SDA电平稳定
        OLED_SCL(1);                      // 拉高SCL，从机在此时刻采样SDA上的数据位
        OLED_Delay();                     // 保持SCL高电平一段时间
        OLED_SCL(0);                      // 拉低SCL，结束本位的传输

        data <<= 1;                       // 数据左移1位，把下一位移到最高位准备发送
    }                                     /* for循环结束，8位数据发送完毕 */

    OLED_SDA_High();                      // 释放SDA，准备接收从机的应答位ACK
    OLED_Delay();                         // 延时等待
    OLED_SCL(1);                          // 第9个时钟拉高，从机在此期间拉低SDA表示应答（此处未读取）
    OLED_Delay();                         // 保持时钟高电平
    OLED_SCL(0);                          // 拉低SCL，结束应答时钟周期
}

static void OLED_WriteCmdBuf(const uint8_t *cmds, uint8_t len)  // 批量写命令：一次I2C事务发送多条命令（控制字节0x00后Co=0，后续字节全是命令）
{
    OLED_I2C_Start();                     // 发送I2C起始信号
    OLED_I2C_WriteByte(OLED_ADDR << 1);   // 发送从机地址（写）
    OLED_I2C_WriteByte(0x00);             // 控制字节0x00：后续全部是命令字节
    while (len--) {                       // 逐条发送命令
        OLED_I2C_WriteByte(*cmds++);      // 发送一条命令，指针后移
    }                                     /* while循环结束 */
    OLED_I2C_Stop();                      // 发送I2C停止信号
}

static void OLED_WriteDataBuf(const uint8_t *data, uint16_t len)  // 批量写显存数据：一次I2C事务发送多个数据字节（比逐字节起停事务快约4倍）
{
    OLED_I2C_Start();                     // 发送I2C起始信号
    OLED_I2C_WriteByte(OLED_ADDR << 1);   // 发送从机地址（写）
    OLED_I2C_WriteByte(0x40);             // 控制字节0x40：后续全部是显存数据，列地址自动加1
    while (len--) {                       // 逐字节发送数据
        OLED_I2C_WriteByte(*data++);      // 发送一个数据字节，指针后移
    }                                     /* while循环结束 */
    OLED_I2C_Stop();                      // 发送I2C停止信号
}

void OLED_WR_CMD(uint8_t cmd)             // 向OLED写入一条命令（控制字节为0x00）
{
    OLED_I2C_Start();                     // 发送I2C起始信号
    OLED_I2C_WriteByte(OLED_ADDR << 1);   // 发送从机地址（左移1位，最低位为0表示写操作）
    OLED_I2C_WriteByte(0x00);             // 发送控制字节0x00，表示接下来传的是命令
    OLED_I2C_WriteByte(cmd);              // 发送具体的命令字节
    OLED_I2C_Stop();                      // 发送I2C停止信号，结束本次传输
}

void OLED_WR_DATA(uint8_t data)           // 向OLED写入一个显示数据字节（控制字节为0x40）
{
    OLED_I2C_Start();                     // 发送I2C起始信号
    OLED_I2C_WriteByte(OLED_ADDR << 1);   // 发送从机地址（左移1位，最低位为0表示写操作）
    OLED_I2C_WriteByte(0x40);             // 发送控制字节0x40，表示接下来传的是显存数据
    OLED_I2C_WriteByte(data);             // 发送具体的显示数据字节（写入显存，列地址自动加1）
    OLED_I2C_Stop();                      // 发送I2C停止信号，结束本次传输
}

void WriteCmd(void)                       // 把初始化命令表里的所有命令发给OLED
{
    OLED_WriteCmdBuf(OLED_INIT_CMD, sizeof(OLED_INIT_CMD));  // 一次I2C事务把整张初始化命令表全部发送完毕
}

void OLED_Init(void)                      // OLED初始化函数：配置引脚并发送初始化命令序列
{
    DL_GPIO_enableOutput(OLED_SCL_PORT, OLED_SCL_PIN);  // 使能SCL引脚为输出模式
    DL_GPIO_enableOutput(OLED_SDA_PORT, OLED_SDA_PIN);  // 使能SDA引脚为输出模式

    /* Enable internal pull-ups in case the module has no external ones */
    DL_GPIO_initDigitalOutputFeatures(GPIO_OLED_OLED_SCL_IOMUX,  // 配置SCL引脚的数字输出特性（SysConfig生成的IOMUX索引）
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,     // 不反相输出，使能内部上拉电阻（防止模块无外接上拉）
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);        // 低驱动强度，禁止高阻态
    /* SDA also needs the input path enabled (for ACK readback) */
    IOMUX->SECCFG.PINCM[GPIO_OLED_OLED_SDA_IOMUX] =              // 直接写SDA引脚的IOMUX引脚控制寄存器
        IOMUX_PINCM_INENA_ENABLE | IOMUX_PINCM_PC_CONNECTED |    // 使能输入通路（读ACK用），引脚连接到外设
        ((uint32_t)0x00000001) | (uint32_t)DL_GPIO_RESISTOR_PULL_UP;  // 选择GPIO功能并使能内部上拉电阻

    OLED_SCL(1);                          // 初始将SCL置为高电平（总线空闲状态）
    OLED_SDA_High();                      // 初始释放SDA为高电平（总线空闲状态）

    delay_cycles(CPUCLK_FREQ / 5U);       // 延时约200ms，等待OLED上电稳定后再初始化

    WriteCmd();                           // 发送SSD1306初始化命令序列
    OLED_Clear();                         // 清屏，清空显存中的残留内容
}

void OLED_Clear(void)                     // 清屏函数：把整个128x64显存全部写0（不显示任何内容）
{
    uint8_t i;                            // 页循环计数器（共8页）
    uint8_t n;                            // 缓冲区填充计数器
    uint8_t buf[32];                      // 全0数据块缓冲区，分4块写满一页（128列）

    for (n = 0; n < 32; n++) {            // 填充缓冲区
        buf[n] = 0x00;                    // 清屏写入的数据全为0
    }                                     /* for填充结束 */

    for (i = 0; i < 8; i++) {             // 遍历8个页（每页8行像素，8页共64行）
        OLED_Set_Pos(0, i);               // 一次事务设置好页地址和第0列
        for (n = 0; n < 4; n++) {         // 每页128列分4块，每块32字节一次事务发送
            OLED_WriteDataBuf(buf, 32);   // 批量写32个0，清除该块的像素点
        }                                 /* 内层for循环结束，本页128列清除完毕 */
    }                                     /* 外层for循环结束，8页全部清除完毕 */
}

void OLED_Display_On(void)                // 开启OLED显示（退出休眠，恢复显示显存内容）
{
    OLED_WR_CMD(0x8D);                    // 电荷泵设置命令
    OLED_WR_CMD(0x14);                    // 使能电荷泵（OLED内部升压，必须开启才能点亮）
    OLED_WR_CMD(0xAF);                    // 0xAF：开启显示
}

void OLED_Display_Off(void)               // 关闭OLED显示（进入休眠，屏幕熄灭但显存内容保留）
{
    OLED_WR_CMD(0x8D);                    // 电荷泵设置命令
    OLED_WR_CMD(0x10);                    // 关闭电荷泵，降低功耗
    OLED_WR_CMD(0xAE);                    // 0xAE：关闭显示
}

void OLED_Set_Pos(uint8_t x, uint8_t y)   // 设置显存写入光标位置：x为列(0~127)，y为页(0~7)
{
    uint8_t cmds[3];                      // 3条定位命令打包，一次I2C事务发完

    cmds[0] = 0xB0 + y;                   // 设置页地址（0xB0~0xB7对应第0~7页）
    cmds[1] = ((x & 0xF0) >> 4) | 0x10;   // 取x的高4位，设置列地址高4位
    cmds[2] = x & 0x0F;                   // 取x的低4位，设置列地址低4位
    OLED_WriteCmdBuf(cmds, 3);            // 一次事务发送3条命令
}

void OLED_On(void)                        // 全屏点亮测试函数：把显存所有列写0x01（每列最下面一个像素点亮）
{
    uint8_t i;                            // 页循环计数器
    uint8_t n;                            // 缓冲区填充计数器
    uint8_t buf[32];                      // 全0x01数据块缓冲区，分4块写满一页

    for (n = 0; n < 32; n++) {            // 填充缓冲区
        buf[n] = 0x01;                    // 点亮该列最低位对应的像素点
    }                                     /* for填充结束 */

    for (i = 0; i < 8; i++) {             // 遍历8个页
        OLED_Set_Pos(0, i);               // 一次事务设置好页地址和第0列
        for (n = 0; n < 4; n++) {         // 每页128列分4块，每块32字节一次事务发送
            OLED_WriteDataBuf(buf, 32);   // 批量写32个0x01
        }                                 /* 内层for循环结束 */
    }                                     /* 外层for循环结束 */
}

static unsigned int oled_pow(uint8_t m, uint8_t n)  // 计算m的n次幂（用于拆数字的各位），仅供本文件内部使用
{
    unsigned int result = 1;              // 累乘结果初始化为1（任何数的0次幂为1）

    while (n--) {                         // 循环n次（后缀自减，先判断后减1）
        result *= m;                      // 结果乘上底数m
    }                                     /* while循环结束 */

    return result;                        // 返回m的n次幂计算结果
}

void OLED_ShowRectangle(uint8_t x, uint8_t y, uint8_t high)  // 在(x,y)处画一条高为high像素的实心竖条（写high个0xFF）
{
    uint8_t n;                            // 循环计数器
    uint8_t buf[32];                      // 全0xFF数据块缓冲区，分块发送

    for (n = 0; n < 32; n++) {            // 填充缓冲区
        buf[n] = 0xFF;                    // 0xFF点亮该列本页全部8个像素
    }                                     /* for填充结束 */

    OLED_Set_Pos(x, y);                   // 把显存光标定位到指定列和页

    while (high >= 32) {                  // 整块发送，每次32字节
        OLED_WriteDataBuf(buf, 32);       // 批量写32个0xFF
        high -= 32;                       // 剩余高度减32
    }                                     /* while整块发送结束 */

    if (high) {                           // 还有不足32字节的零头
        OLED_WriteDataBuf(buf, high);     // 把零头一次发完
    }                                     /* if零头处理结束 */
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)  // 在(x,y)处显示一个ASCII字符，Char_Size为字号16或8
{
    uint8_t c;                            // 字符在字模表中的索引

    if (chr < ' ' || chr > '~') {         // 字符超出可显示ASCII范围（空格到~）
        chr = ' ';                        // 用空格代替，防止越界访问字模表
    }                                     /* if结束 */

    c = chr - ' ';                        // 字模表从空格开始存储，减去空格得到表内索引

    if (x > 127) {                        // 列坐标超出屏幕右边界
        x = 0;                            // 回到行首
        y += 2;                           // 页坐标下移2页（即换行）
    }                                     /* if结束 */

    if (Char_Size == 16) {                // 使用8x16字号（宽8列、高16像素，占2页）
        OLED_Set_Pos(x, y);               // 光标定位到上半部分（第y页）
        OLED_WriteDataBuf(&F8X16[c * 16], 8);      // 上半部分8列字模一次事务写入显存

        OLED_Set_Pos(x, y + 1);           // 光标定位到下半部分（第y+1页）
        OLED_WriteDataBuf(&F8X16[c * 16 + 8], 8);  // 下半部分8列字模一次事务写入显存
    } else {                              // 否则使用6x8字号（宽6列、高8像素，占1页）
        OLED_Set_Pos(x, y);               // 光标定位到指定位置
        OLED_WriteDataBuf(F6x8[c], 6);    // 6列字模一次事务写入显存
    }                                     /* if-else结束 */
}

void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size)  // 从(x,y)开始显示一个字符串，自动换行
{
    uint8_t j = 0;                        // 字符串字符索引，从第一个字符开始
    uint8_t step;                         // 每个字符占用的列宽（步进）

    step = (Char_Size == 16) ? 8 : 6;     // 8x16字号宽8列，6x8字号宽6列

    while (chr[j] != '\0') {              // 遍历字符串直到遇到结束符'\0'
        OLED_ShowChar(x, y, chr[j], Char_Size);  // 显示当前字符

        x += step;                        // 列坐标右移一个字符宽度
        if (x > 120) {                    // 接近屏幕右边界，需要换行
            x = 0;                        // 回到行首
            y += (Char_Size == 16) ? 2 : 1;  // 16号字占2页下移2页，8号字占1页下移1页
        }                                 /* if结束 */

        j++;                              // 处理字符串的下一个字符
    }                                     /* while循环结束，字符串显示完毕 */
}

void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2)  // 显示无符号数字，len为位数，高位补空格
{
    uint8_t t;                            // 当前处理的位序号（0为最高位）
    uint8_t temp;                         // 当前位的数字值
    uint8_t enshow = 0;                   // 前导零消除标志：0表示还在前导零区域

    for (t = 0; t < len; t++) {           // 从最高位到最低位逐位处理
        temp = (num / oled_pow(10, len - t - 1)) % 10;  // 取出第t位的数字（除以10的幂再对10取余）

        if (enshow == 0 && t < (len - 1)) {  // 还没遇到非零位，且不是最后一位
            if (temp == 0) {              // 当前位是前导零
                OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2);  // 前导零显示为空格
                continue;                 // 跳过本次循环，处理下一位
            } else {                      // 遇到第一个非零数字
                enshow = 1;               // 置标志，之后即使为0也要正常显示
            }                             /* 内层if-else结束 */
        }                                 /* 外层if结束 */

        OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2);  // 数字加'0'转成ASCII码显示，x按半字号宽度偏移
    }                                     /* for循环结束 */
}

void OLED_ShowSignedNum(uint8_t x, uint8_t y, int num, uint8_t len, uint8_t size2)  // 显示带符号数字，先显示+/-号再显示数字
{
    uint8_t t;                            // 当前处理的位序号
    uint8_t temp;                         // 当前位的数字值
    uint8_t enshow = 0;                   // 前导零消除标志

    if (num >= 0) {                       // 非负数
        OLED_ShowChar(x, y, '+', size2);  // 在最前面显示正号'+'
    } else {                              // 负数
        OLED_ShowChar(x, y, '-', size2);  // 在最前面显示负号'-'
        num = -num;                       // 取绝对值，后面按无符号数处理
    }                                     /* if-else结束 */

    for (t = 0; t < len; t++) {           // 从最高位到最低位逐位处理
        temp = (num / oled_pow(10, len - t - 1)) % 10;  // 取出第t位的数字

        if (enshow == 0 && t < (len - 1)) {  // 前导零区域且不是最后一位
            if (temp == 0) {              // 当前位是前导零
                OLED_ShowChar(x + (size2 / 2) * (t + 1), y, ' ', size2);  // 显示空格（t+1是给符号位让出一个字符位置）
                continue;                 // 跳过本次循环
            } else {                      // 遇到第一个非零数字
                enshow = 1;               // 置标志，后续数字正常显示
            }                             /* 内层if-else结束 */
        }                                 /* 外层if结束 */

        OLED_ShowChar(x + (size2 / 2) * (t + 1), y, temp + '0', size2);  // 显示该位数字（t+1偏移是为符号位留位）
    }                                     /* for循环结束 */
}

void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)  // 显示一个16x16点阵汉字，no为汉字在Hzk字库中的序号
{
    OLED_Set_Pos(x, y);                   // 光标定位到汉字上半部分（第y页）
    OLED_WriteDataBuf(Hzk[2 * no], 16);   // 上半部分16列字模一次事务写入显存（Hzk中2*no为上半）

    OLED_Set_Pos(x, y + 1);               // 光标定位到汉字下半部分（第y+1页）
    OLED_WriteDataBuf(Hzk[2 * no + 1], 16);  // 下半部分16列字模一次事务写入显存（2*no+1为下半）
}

void OLED_Draw12864BMP(uint8_t num)       // 全屏显示一张128x64位图，num为图片在BMP数组中的编号（从1开始）
{
    uint8_t y;                            // 页循环计数器

    if (num == 0) {                       // 编号0不对应任何图片
        return;                           // 直接返回，不做任何操作
    }                                     /* if结束 */

    for (y = 0; y < 8; y++) {             // 遍历8个页
        OLED_Set_Pos(0, y);               // 光标定位到本页第0列
        OLED_WriteDataBuf(&BMP[num - 1][y * 128], 128);  // 本页128列位图数据一次事务写入显存（线性索引，对应DISPLAY_MODE=0的取模方式）
    }                                     /* 外层for循环结束，整屏图片显示完毕 */
}

void OLED_ShowFloat2(uint8_t x, uint8_t y, float value, uint8_t int_len, uint8_t size)
{
    int32_t value_x100;
    uint8_t pos = x;
    uint8_t char_width = size / 2;

    if (value < 0.0f) {
        OLED_ShowChar(pos, y, '-', size);
        value = -value;
    } else {
        OLED_ShowChar(pos, y, ' ', size);
    }

    pos += char_width;

    value_x100 = (int32_t)(value * 100.0f + 0.5f);

    OLED_ShowNum(pos, y, value_x100 / 100, int_len, size);
    pos += char_width * int_len;

    OLED_ShowChar(pos, y, '.', size);
    pos += char_width;

    OLED_ShowChar(pos, y, (value_x100 / 10) % 10 + '0', size);
    OLED_ShowChar(pos + char_width, y, value_x100 % 10 + '0', size);
}
