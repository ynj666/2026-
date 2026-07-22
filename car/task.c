#include "task.h"                 // 包含本模块自己的头文件，拿到 TASK_NUM 和函数声明
#include "ti_msp_dl_config.h"     // 包含 SysConfig 生成的配置头文件，拿到芯片寄存器定义（NVIC_SystemReset 用）
#include "control.h"              // 包含控制模块头文件，提供 Control_StartPosition、Control_Enable、Control_PositionDone 等接口
#include "motor.h"                // 包含电机模块头文件，提供 Motor_StopAll 停车接口

#define TASK1_TARGET_PULSES  20000    // 任务1目标里程：两侧各走20000个编码器脉冲（只用编码器走直线）

static uint8_t task_current = 0;            // 当前选中的任务号：上电默认为0（未选择），按键1每按一次加1进入1~5循环，超过5回到1
static uint8_t task_running = 0;          // 任务运行状态：1=执行中，0=暂停（按键2切换）

/* ===== 五个任务的启动函数：要开始干什么写在对应函数里 ===== */
static void Task1_Start(void)     // 任务1：只用编码器走直线，两侧各走 TASK1_TARGET_PULSES 个脉冲
{
    Control_StartPosition(TASK1_TARGET_PULSES, TASK1_TARGET_PULSES);  // 启动位置+速度双闭环，左右侧目标相同即走直线，到位自动停车
}                                 // Task1_Start 函数结束

static void Task2_Start(void)     // 任务2
{
    /* TODO: 在这里写任务2开始要做的事 */
}                                 // Task2_Start 函数结束

static void Task3_Start(void)     // 任务3
{
    /* TODO: 在这里写任务3开始要做的事 */
}                                 // Task3_Start 函数结束

static void Task4_Start(void)     // 任务4
{
    /* TODO: 在这里写任务4开始要做的事 */
}                                 // Task4_Start 函数结束

static void Task5_Start(void)     // 任务5
{
    /* TODO: 在这里写任务5开始要做的事 */
}                                 // Task5_Start 函数结束

void Task_Init(void)              // 任务模块初始化：上电后调用一次
{
    task_current = 0;             // 当前任务号恢复默认值0（未选择任务，按按键2不会执行任何动作）
    task_running = 0;             // 初始为暂停状态，等待按键2启动
}                                 // Task_Init 函数结束

uint8_t Task_GetCurrent(void)     // 读取当前选中的任务号（1~5）
{
    return task_current;          // 返回当前任务号
}                                 // Task_GetCurrent 函数结束

uint8_t Task_GetRunning(void)     // 读取任务运行状态
{
    return task_running;          // 返回 1=执行中，0=暂停
}                                 // Task_GetRunning 函数结束

void Task_SwitchNext(void)        // 切换任务：任务号加1，超过 TASK_NUM 回到1（按键1调用）
{
    task_current++;               // 任务号加1
    if (task_current > TASK_NUM) {// 超过最大任务号5
        task_current = 1;         // 回到任务1，循环切换
    }                             // if 判断结束
}                                 // Task_SwitchNext 函数结束

void Task_ToggleRun(void)         // 开始/暂停切换（按键2调用）：奇数次按下开始，偶数次按下暂停
{
    if (task_running) {           // 当前在执行中：本次按下为暂停
        task_running = 0;         // 状态切回暂停
        Control_Enable(0);        // 退出闭环，清积分
        Motor_StopAll();          // 四轮立即停车
    } else {                      // 当前在暂停：本次按下为开始
        if (task_current == 0) {  // 任务号0表示还未选择任务
            return;               // 不做任何事，保持暂停状态（防止空任务也显示执行中）
        }                         // if 未选任务判断结束
        task_running = 1;         // 状态切为执行中
        switch (task_current) {   // 按当前任务号分发启动动作
            case 1:  Task1_Start(); break;  // 启动任务1
            case 2:  Task2_Start(); break;  // 启动任务2
            case 3:  Task3_Start(); break;  // 启动任务3
            case 4:  Task4_Start(); break;  // 启动任务4
            case 5:  Task5_Start(); break;  // 启动任务5
            default: break;       // 任务号异常时不做任何事
        }                         // switch 分发结束
    }                             // if-else 状态切换结束
}                                 // Task_ToggleRun 函数结束

void Task_Tick(void)              // 任务状态巡检：主循环周期调用，检测任务是否已完成
{
    if (task_running && task_current == 1) {    // 任务1执行中
        if (Control_PositionDone()) {           // 位置闭环已到位停车
            task_running = 0;                   // 自动回到暂停态，屏幕显示暂停
        }                                       // if 到位判断结束
    }                             // if 任务1巡检结束
    /* 其他任务如有"完成"条件，在这里照样子补 */
}                                 // Task_Tick 函数结束

void Task_SoftReset(void)         // 软件归零复位（按键3调用）：触发芯片系统复位
{
    /* 直接复位整个MCU：所有变量、外设、里程、姿态全都回到刚上电时的状态，
       等效于按了一次复位键，之后程序从头重新运行 */
    NVIC_SystemReset();           // Cortex-M 系统复位请求，此函数不会返回
}                                 // Task_SoftReset 函数结束
