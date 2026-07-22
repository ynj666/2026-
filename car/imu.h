#ifndef IMU_H                       // 头文件保护：防止IMU_H被重复包含导致重定义错误
#define IMU_H                       // 定义包含标记，第二次包含时#ifndef为假，内容被跳过

#include <stdint.h>                 // 标准整数类型头文件，提供int8_t/int16_t/uint8_t/uint16_t等定长类型

#define IMU_OK                  0   // 函数返回值：0表示操作成功
#define IMU_NOT_READY           1   // 函数返回值：1表示传感器数据未准备好或校准无有效采样
#define IMU_CAL_FALLBACK        2   // 函数返回值：2表示校准多次检测到移动，已改用内置兜底零偏（系统可继续运行）

#define IMU_ACC_LSB_PER_G       16384.0f  // 加速度计灵敏度：±2g量程下每g对应16384个LSB，原始值除以它得g单位
#define IMU_GYR_LSB_PER_DPS     16.384f   // 陀螺仪灵敏度：±2000°/s量程下每(°/s)对应16.384个LSB

typedef struct {                    // 定义结构体类型IMU_RawData：存放传感器原始整数读数
    int16_t acc_x;                  // 加速度X轴原始值（LSB，16位有符号）
    int16_t acc_y;                  // 加速度Y轴原始值
    int16_t acc_z;                  // 加速度Z轴原始值
    int16_t gyr_x;                  // 陀螺仪X轴原始值
    int16_t gyr_y;                  // 陀螺仪Y轴原始值
    int16_t gyr_z;                  // 陀螺仪Z轴原始值
} IMU_RawData;                      // 结构体定义结束，类型名为IMU_RawData

typedef struct {                    // 定义结构体类型IMU_Data：存放换算后的物理量和解算姿态角
    float acc_x_g;                  // 加速度X轴，单位g
    float acc_y_g;                  // 加速度Y轴，单位g
    float acc_z_g;                  // 加速度Z轴，单位g

    float gyr_x_dps;                // 陀螺仪X轴角速度，单位°/s（已减零偏、过死区）
    float gyr_y_dps;                // 陀螺仪Y轴角速度，单位°/s
    float gyr_z_dps;                // 陀螺仪Z轴角速度，单位°/s

    float roll_acc;                 // 仅用加速度计估算的横滚角（°），静止时准确，可用于对比校验
    float pitch_acc;                // 仅用加速度计估算的俯仰角（°）

    float yaw;                      // Mahony融合后的偏航角（°），范围±180°，已减零点偏移
    float pitch;                    // 融合后的俯仰角（°），平衡车主控用这个角度
    float roll;                     // 融合后的横滚角（°）
} IMU_Data;                         // 结构体定义结束，类型名为IMU_Data

int8_t IMU_Init(void);              // 初始化IMU：SPI+芯片+传感器配置使能，返回IMU_OK(0)成功
int8_t IMU_Calibrate(uint16_t samples);  // 静止校准：采样samples次求零偏和姿态零点，传0默认200次；带静止检测（移动则重试3次），多次失败用兜底零偏并返回IMU_CAL_FALLBACK
int8_t IMU_ReadRaw(IMU_RawData *data);   // 读取原始整数数据到data，返回IMU_OK成功/IMU_NOT_READY未就绪
int8_t IMU_ReadData(IMU_Data *data);     // 以默认周期10ms读取并融合解算姿态，结果填入data
int8_t IMU_ReadDataDt(IMU_Data *data, float dt_s);  // 按指定周期dt_s（秒）读取并融合解算姿态
void IMU_ResetYaw(float yaw_deg);   // 把当前偏航角重设为yaw_deg（一般传0实现yaw清零）
uint8_t IMU_DebugGetChipId(void);   // 调试用：返回BMI270芯片ID，正常值为0x24

#endif                              // 头文件保护结束，对应开头的#ifndef IMU_H
