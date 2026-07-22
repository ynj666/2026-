#include "bmi270_spi.h"             // 包含本模块头文件，声明SPI读写接口函数
#include "ti_msp_dl_config.h"       // 包含SysConfig生成的驱动库配置头文件（SPI_0_INST、GPIO端口、CPUCLK_FREQ等定义）

#define BMI270_SPI_INST     SPI_0_INST      // BMI270所使用的SPI外设实例（SPI0）
#define BMI270_CS_PORT      GPIOB           // 片选CS引脚所在的GPIO端口（GPIOB）
#define BMI270_CS_PIN       DL_GPIO_PIN_6   // 片选CS引脚编号（PB6）

#define BMI270_REG_CHIP_ID  0x00    // BMI270芯片ID寄存器地址（读出应为0x24）
#define BMI270_READ_MASK    0x80    // SPI读操作标志：寄存器地址最高位置1表示读

static void BMI270_SPI_CS_Low(void) // 拉低片选，选中BMI270开始SPI通信
{
    DL_GPIO_clearPins(BMI270_CS_PORT, BMI270_CS_PIN);   // CS引脚输出低电平，选中芯片
}   /* BMI270_SPI_CS_Low函数结束 */

static void BMI270_SPI_CS_High(void)    // 拉高片选，结束与BMI270的SPI通信
{
    DL_GPIO_setPins(BMI270_CS_PORT, BMI270_CS_PIN);     // CS引脚输出高电平，释放芯片
}   /* BMI270_SPI_CS_High函数结束 */

static void BMI270_SPI_ClearRx(void)    // 清空SPI接收FIFO，丢弃上次通信残留的数据
{
    while (!DL_SPI_isRXFIFOEmpty(BMI270_SPI_INST)) {    // 只要接收FIFO非空就循环读取
        (void)DL_SPI_receiveData8(BMI270_SPI_INST);     // 读出1字节并丢弃，达到清空FIFO的目的
    }   /* while清空接收FIFO循环结束 */
}   /* BMI270_SPI_ClearRx函数结束 */

static uint8_t BMI270_SPI_Transfer(uint8_t tx)  // SPI全双工传输一个字节：发送tx，返回同时收到的字节
{
    DL_SPI_transmitDataBlocking8(BMI270_SPI_INST, tx);  // 阻塞式发送一个字节（等待发送FIFO可写）

    while (DL_SPI_isRXFIFOEmpty(BMI270_SPI_INST));  // 等待接收FIFO收到从机返回的字节

    return DL_SPI_receiveData8(BMI270_SPI_INST);    // 读取并返回接收到的字节
}   /* BMI270_SPI_Transfer函数结束 */

void BMI270_SPI_DelayUs(uint32_t period, void *intf_ptr)    // 微秒级延时函数，作为BMI270官方驱动库的延时回调
{
    (void)intf_ptr;                                     // intf_ptr参数未使用，强制转换消除编译器警告
    delay_cycles((CPUCLK_FREQ / 1000000U) * period);    // 按CPU主频换算出每微秒的时钟周期数，延时period微秒
}   /* BMI270_SPI_DelayUs函数结束 */

void BMI270_SPI_Init(void)  // BMI270的SPI接口初始化
{
    BMI270_SPI_CS_High();           // 片选默认拉高，不选中芯片
    BMI270_SPI_DelayUs(10000, 0);   // 延时10ms，等待BMI270上电稳定
}   /* BMI270_SPI_Init函数结束 */

uint8_t BMI270_SPI_ReadID(void) // 读取BMI270芯片ID（正常应返回0x24，用于检测芯片是否在线）
{
    uint8_t id;                     // 存放读回的芯片ID

    BMI270_SPI_ClearRx();           // 先清空接收FIFO，防止旧数据干扰本次读取
    BMI270_SPI_CS_Low();            // 拉低片选，选中芯片开始通信

    (void)BMI270_SPI_Transfer(BMI270_REG_CHIP_ID | BMI270_READ_MASK);   // 发送ID寄存器地址（带读标志），返回的字节丢弃
    (void)BMI270_SPI_Transfer(0x00);    // BMI270的SPI读时序需要一个空转(dummy)字节，读回内容丢弃
    id = BMI270_SPI_Transfer(0x00);     // 再发一个dummy字节，这次读回的才是真正的芯片ID

    while (DL_SPI_isBusy(BMI270_SPI_INST)); // 等待SPI控制器空闲，确保最后一个字节已发送完成
    BMI270_SPI_CS_High();           // 拉高片选，结束通信

    return id;                      // 返回读到的芯片ID
}   /* BMI270_SPI_ReadID函数结束 */

BMI2_INTF_RETURN_TYPE BMI270_SPI_Read(uint8_t reg_addr, uint8_t *reg_data,
                                      uint32_t len, void *intf_ptr) // 驱动库读回调：从reg_addr寄存器连续读取len字节存入reg_data
{
    (void)intf_ptr;                             // intf_ptr参数未使用，消除编译警告

    BMI270_SPI_ClearRx();                       // 清空接收FIFO中的残留数据
    BMI270_SPI_CS_Low();                        // 拉低片选，开始SPI通信

    (void)BMI270_SPI_Transfer(reg_addr | BMI270_READ_MASK); // 发送寄存器地址，最高位置1表示读操作

    while (len--) {                             // 循环len次，逐字节读取
        *reg_data++ = BMI270_SPI_Transfer(0x00);    // 发送dummy字节换回应答数据，存入缓冲区后指针后移
    }   /* while连续读取循环结束 */

    while (DL_SPI_isBusy(BMI270_SPI_INST));     // 等待SPI总线空闲（最后一字节传输完成）
    BMI270_SPI_CS_High();                       // 拉高片选，结束通信

    return BMI2_INTF_RET_SUCCESS;               // 返回接口操作成功
}   /* BMI270_SPI_Read函数结束 */

BMI2_INTF_RETURN_TYPE BMI270_SPI_Write(uint8_t reg_addr, const uint8_t *reg_data,
                                       uint32_t len, void *intf_ptr)    // 驱动库写回调：向reg_addr寄存器连续写入len字节
{
    (void)intf_ptr;                             // intf_ptr参数未使用，消除编译警告

    BMI270_SPI_ClearRx();                       // 清空接收FIFO中的残留数据
    BMI270_SPI_CS_Low();                        // 拉低片选，开始SPI通信

    (void)BMI270_SPI_Transfer(reg_addr);        // 发送寄存器地址（最高位为0表示写操作）

    while (len--) {                             // 循环len次，逐字节写入
        (void)BMI270_SPI_Transfer(*reg_data++); // 取缓冲区数据发送出去，指针后移，返回字节丢弃
    }   /* while连续写入循环结束 */

    while (DL_SPI_isBusy(BMI270_SPI_INST));     // 等待SPI总线空闲（最后一字节发送完成）
    BMI270_SPI_CS_High();                       // 拉高片选，结束通信

    return BMI2_INTF_RET_SUCCESS;               // 返回接口操作成功
}   /* BMI270_SPI_Write函数结束 */