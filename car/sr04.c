#include "sr04.h"  // 包含本模块头文件，获得SR04函数声明和SR04_TIMEOUT_CM宏
#include "ti_msp_dl_config.h"  // 包含SysConfig生成的配置头文件，提供CAPTURE_ECHO_INST、GPIO_OUTPUT_TRIG等外设定义

#define SR04_TIMER_HZ        10000000U  // 捕获定时器计数频率10MHz，即每计1个数代表0.1us
#define SR04_TICKS_PER_US    (SR04_TIMER_HZ / 1000000U)  // 每微秒对应的计数个数：10
#define SR04_TICKS_PER_CM    (SR04_TICKS_PER_US * 58U)  // 每厘米距离对应的计数个数（超声波往返1cm约58us），即580
#define SR04_PERIOD_TICKS    (CAPTURE_ECHO_INST_LOAD_VALUE + 1U)  // 定时器一个完整周期的计数总数（重载值+1，因为向下计数到0）
#define SR04_MAX_CM          450U  // 有效测量上限450cm，超过视为无效
#define SR04_TIMEOUT_MS      50U  // 等待回波的超时时间50ms，超时说明前方没有反射物

static volatile uint8_t sr04_ready = 0;  // 测量完成标志：中断里置1表示本次测距完成（volatile防止被编译器优化掉）
static volatile uint16_t sr04_distance_cm = SR04_TIMEOUT_CM;  // 最近一次测得的距离（厘米），初值为超时无效值

static void SR04_DelayUs(uint32_t us)  // 微秒级软件延时
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * us);  // 按CPU主频换算出指定微秒数对应的时钟周期数并空转等待
}

static void SR04_DelayMs(uint32_t ms)  // 毫秒级软件延时
{
    delay_cycles((CPUCLK_FREQ / 1000U) * ms);  // 按CPU主频换算出指定毫秒数对应的时钟周期数并空转等待
}

static void SR04_Trigger(void)  // 给TRIG引脚发一个触发脉冲，启动一次超声波测距
{
    DL_GPIO_clearPins(GPIO_OUTPUT_TRIG_PORT, GPIO_OUTPUT_TRIG_PIN);  // TRIG先拉低，保证脉冲起点干净
    SR04_DelayUs(2);  // 保持低电平2us

    DL_GPIO_setPins(GPIO_OUTPUT_TRIG_PORT, GPIO_OUTPUT_TRIG_PIN);  // TRIG拉高
    SR04_DelayUs(10);  // 保持高电平10us（HC-SR04要求触发脉宽>=10us）

    DL_GPIO_clearPins(GPIO_OUTPUT_TRIG_PORT, GPIO_OUTPUT_TRIG_PIN);  // TRIG拉低，触发脉冲结束，模块开始发射超声波
}

void SR04_Init(void)  // SR04超声波模块初始化：配置定时器捕获ECHO引脚回波脉宽
{
    static DL_TimerG_CaptureConfig sr04_capture_config = {  // 定时器捕获配置结构体（static只初始化一次）
        .captureMode  = DL_TIMER_CAPTURE_MODE_PULSE_WIDTH,  // 捕获模式：脉宽测量模式，直接测出ECHO高电平宽度
        .period       = CAPTURE_ECHO_INST_LOAD_VALUE,  // 定时器重载值，决定可测的最大脉宽
        .startTimer   = DL_TIMER_STOP,  // 初始化后不立即启动计数，等测距时再启动
        .edgeCaptMode = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,  // 从上升沿开始检测（回波开始）
        .inputChan    = DL_TIMER_INPUT_CHAN_0,  // 使用捕获通道0（CC0）连接ECHO引脚
        .inputInvMode = DL_TIMER_CC_INPUT_INV_NOINVERT,  // 输入不反相
    };  /* 捕获配置结构体定义结束 */

    DL_GPIO_clearPins(GPIO_OUTPUT_TRIG_PORT, GPIO_OUTPUT_TRIG_PIN);  // TRIG引脚初始置低，避免误触发

    DL_TimerG_stopCounter(CAPTURE_ECHO_INST);  // 先停止定时器，再改配置
    DL_TimerG_initCaptureMode(CAPTURE_ECHO_INST, &sr04_capture_config);  // 把上面的捕获配置写入定时器寄存器
    DL_TimerG_enableInterrupt(CAPTURE_ECHO_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);  // 使能CC0捕获完成中断，测到回波时进中断
    DL_TimerG_clearInterruptStatus(CAPTURE_ECHO_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);  // 清除可能残留的中断标志，防止误触发中断

    NVIC_ClearPendingIRQ(CAPTURE_ECHO_INST_INT_IRQN);  // 清除NVIC中该定时器中断的挂起状态
    NVIC_EnableIRQ(CAPTURE_ECHO_INST_INT_IRQN);  // 在NVIC中使能该定时器中断，允许CPU响应
}

uint16_t SR04_ReadCm(void)  // 执行一次测距，返回距离（厘米），超时返回SR04_TIMEOUT_CM
{
    uint32_t timeout = SR04_TIMEOUT_MS;  // 剩余超时时间（毫秒），每等1ms减1

    sr04_ready = 0;  // 清除完成标志，表示本次测量还没出结果
    sr04_distance_cm = SR04_TIMEOUT_CM;  // 预置结果为无效值，若超时则返回它

    DL_TimerG_stopCounter(CAPTURE_ECHO_INST);  // 停止定时器，准备开始新一轮测量
    DL_TimerG_clearInterruptStatus(CAPTURE_ECHO_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);  // 清除旧的捕获中断标志
    DL_TimerG_startCounter(CAPTURE_ECHO_INST);  // 启动定时器，开始等待ECHO回波

    SR04_Trigger();  // 发触发脉冲，让模块发射超声波

    while ((sr04_ready == 0) && (timeout > 0)) {  // 等待中断置位完成标志，或超时退出
        SR04_DelayMs(1);  // 每次等1ms
        timeout--;  // 剩余超时时间减1
    }  /* while等待循环结束 */

    DL_TimerG_stopCounter(CAPTURE_ECHO_INST);  // 本次测量结束，停止定时器

    return sr04_distance_cm;  // 返回测得的距离，超时则为SR04_TIMEOUT_CM(0xFFFF)
}

void CAPTURE_ECHO_INST_IRQHandler(void)  // 捕获定时器中断服务函数：ECHO回波捕获完成时进入
{
    switch (DL_TimerG_getPendingInterrupt(CAPTURE_ECHO_INST)) {  // 读取并判断是哪个中断源触发的
        case DL_TIMER_IIDX_CC0_DN: {  // CC0捕获事件（ECHO脉宽测量完成）
            uint32_t cap = DL_TimerG_getCaptureCompareValue(  // 读取捕获寄存器值，即脉宽对应的计数值
                CAPTURE_ECHO_INST, DL_TIMER_CC_0_INDEX);  // 指定定时器实例和捕获通道0

            uint32_t width_ticks = cap;  // 回波高电平宽度的计数个数，先直接取捕获值

            if (cap > (SR04_PERIOD_TICKS / 2U)) {  // 捕获值大于半周期说明是向下计数的回绕情况
                width_ticks = SR04_PERIOD_TICKS - cap;  // 用周期值减去捕获值得到真实的脉宽计数
            }  /* if回绕判断结束 */

            if (width_ticks > (SR04_MAX_CM * SR04_TICKS_PER_CM)) {  // 脉宽换算超过最大量程，视为无效回波
                sr04_distance_cm = SR04_TIMEOUT_CM;  // 距离记为无效值
            } else {  // 脉宽在有效范围内
                sr04_distance_cm =  // 把计数个数换算成厘米（加半周期做四舍五入）并存入结果
                    (uint16_t)((width_ticks + SR04_TICKS_PER_CM / 2U) / SR04_TICKS_PER_CM);
            }  /* if-else量程判断结束 */

            sr04_ready = 1;  // 置位完成标志，通知SR04_ReadCm本次测距已出结果
            break;  // 本中断源处理完毕，跳出switch
        }  /* case CC0捕获事件处理结束 */

        default:  // 其他未处理的中断源
            break;  // 不做处理，直接忽略
    }  /* switch中断源判断结束 */
}