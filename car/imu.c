#include "imu.h"              // 本模块头文件：包含IMU数据结构体定义与函数声明
#include "bmi270_spi.h"       // BMI270底层SPI读写接口（读寄存器/写寄存器/微秒延时）
#include "bmi270.h"           // BMI270传感器驱动库（Bosch官方驱动，提供bmi270_init等）
#include "bmi2_defs.h"        // BMI2驱动库的寄存器地址、宏常量、结构体类型定义
#include "bmi2.h"             // BMI2通用驱动接口（bmi2_get_sensor_data等通用API）
#include <string.h>           // 标准库字符串/内存操作函数，本文件用到memset
#include <math.h>             // 标准库数学函数：sqrtf开方、atan2f反正切、asinf反正弦、fabsf绝对值

#define IMU_READ_WRITE_LEN      46              // SPI单次读写最大字节数，BMI270突发读取需要足够长度
#define IMU_DEFAULT_DT_S        0.01f           // 默认姿态解算周期0.01秒（即100Hz），用于四元数积分
#define IMU_PI                  3.14159265358979323846f  // 圆周率常量（当前代码未使用，留作备用）
#define IMU_RAD_TO_DEG          57.2957795f     // 弧度转角度系数：180/π≈57.2958
#define IMU_DEG_TO_RAD          0.0174532925f   // 角度转弧度系数：π/180≈0.0174533

#define IMU_MAHONY_KP           0.5f            // Mahony姿态融合比例增益：加速度计纠偏的快慢，越大收敛越快但越易受振动干扰
#define IMU_MAHONY_KI           0.001f          // Mahony姿态融合积分增益：缓慢补偿陀螺仪零偏，越小越稳
#define IMU_CAL_DELAY_US        5000U           // 校准时每两次采样之间的延时5000微秒（5ms，对应200Hz采样）
#define IMU_CAL_SKIP_SAMPLES    10              // 校准时丢弃的头部样本数：刚开始数据未稳定，不参与平均
#define IMU_CAL_STILL_RANGE_DPS 2.0f            // 静止判定极差阈值：采样期间单轴角速度最大值-最小值超过2°/s就认为车在动
#define IMU_CAL_MAX_RETRY       3               // 校准最大重试次数：检测到移动就等一会重新采样
#define IMU_CAL_RETRY_DELAY_US  500000U         // 每次重试前的等待时间500ms，给手放下车的时间
/* 兜底零偏：连续多次校准都检测到移动时使用。
   用法：挑一次成功校准后串口打印的零偏值填到这里，以后即使上电环境不理想也能正常跑 */
#define IMU_GYRO_OFFSET_FALLBACK_X  0.0f        // 陀螺仪X轴兜底零偏（°/s），填实测值
#define IMU_GYRO_OFFSET_FALLBACK_Y  0.0f        // 陀螺仪Y轴兜底零偏（°/s），填实测值
#define IMU_GYRO_OFFSET_FALLBACK_Z  0.0f        // 陀螺仪Z轴兜底零偏（°/s），填实测值
/* 陀螺仪零速死区：校零后低于此角速度视为静止，直接清零，
   减缓静止/微振动时残余零偏与噪声积分造成的yaw漂移 */
#define IMU_GYRO_DEADBAND_DPS   0.5f            // 陀螺仪死区阈值0.5°/s：低于此角速度直接当作0处理
/* 静止判定阈值：三轴角速度都小于此值且加速度模长接近1g（无振动无加速） */
#define IMU_STILL_GYRO_DPS      1.0f            // 静止判定角速度阈值1.0°/s：三轴角速度都小于它才可能静止
#define IMU_STILL_ACC_G         0.1f            // 静止判定加速度阈值0.1g：加速度模长与1g的偏差小于它才算无加速
/* 静止时零偏在线跟踪系数：越小越慢越稳，0.005约对应2秒时间常数@100Hz */
#define IMU_BIAS_ALPHA          0.005f          // 零偏在线跟踪系数：每次静止时把残余角速度的0.5%回灌到零偏估计中

static struct bmi2_dev imu_dev;                 // BMI270设备句柄（静态全局），保存接口函数指针和芯片状态
static uint8_t imu_is_init = 0;                 // IMU初始化成功标志：0未初始化，1已初始化，用于防止未初始化就读数据

static float imu_gyr_x_offset = 0.0f;           // 陀螺仪X轴零偏（°/s），校准时求出，读数时减去
static float imu_gyr_y_offset = 0.0f;           // 陀螺仪Y轴零偏（°/s）
static float imu_gyr_z_offset = 0.0f;           // 陀螺仪Z轴零偏（°/s）

static float imu_roll_offset = 0.0f;            // 横滚角roll的零点偏移（°），校准时把当前姿态设为0°
static float imu_pitch_offset = 0.0f;           // 俯仰角pitch的零点偏移（°）
static float imu_yaw_offset = 0.0f;             // 偏航角yaw的零点偏移（°），由IMU_ResetYaw设置

static float imu_q0 = 1.0f;                     // 姿态四元数分量q0（标量部），初始为单位四元数表示无旋转
static float imu_q1 = 0.0f;                     // 姿态四元数分量q1（X轴部）
static float imu_q2 = 0.0f;                     // 姿态四元数分量q2（Y轴部）
static float imu_q3 = 0.0f;                     // 姿态四元数分量q3（Z轴部）

static float imu_ex_int = 0.0f;                 // Mahony算法误差积分项X分量，用于积分补偿陀螺零偏
static float imu_ey_int = 0.0f;                 // Mahony算法误差积分项Y分量
static float imu_ez_int = 0.0f;                 // Mahony算法误差积分项Z分量

static int8_t IMU_ReadDataNoOffset(IMU_Data *data);  // 前向声明：读取原始数据并换算成物理单位（不减零偏），供本文件内部使用

static float IMU_Wrap180(float deg)             // 静态函数：把角度归一化到(-180, 180]区间
{                                               // IMU_Wrap180函数体开始
    while (deg > 180.0f) deg -= 360.0f;         // 角度超过180°就不断减360°，直到落入区间内
    while (deg < -180.0f) deg += 360.0f;        // 角度小于-180°就不断加360°，直到落入区间内
    return deg;                                 // 返回归一化后的角度
}                                               // IMU_Wrap180函数结束

static float IMU_Clamp(float value, float min, float max)  // 静态函数：把数值限制在[min, max]范围内（限幅）
{                                               // IMU_Clamp函数体开始
    if (value < min) return min;                // 小于下限则返回下限
    if (value > max) return max;                // 大于上限则返回上限
    return value;                               // 范围内则原样返回
}                                               // IMU_Clamp函数结束

static void IMU_ResetAHRS(void)                 // 静态函数：复位姿态解算（AHRS）状态，四元数和误差积分清零
{                                               // IMU_ResetAHRS函数体开始
    imu_q0 = 1.0f;                              // 四元数恢复为单位四元数q0=1（表示无旋转的初始姿态）
    imu_q1 = 0.0f;                              // 四元数q1清零
    imu_q2 = 0.0f;                              // 四元数q2清零
    imu_q3 = 0.0f;                              // 四元数q3清零

    imu_ex_int = 0.0f;                          // 误差积分项X清零，重新积累零偏补偿
    imu_ey_int = 0.0f;                          // 误差积分项Y清零
    imu_ez_int = 0.0f;                          // 误差积分项Z清零
}                                               // IMU_ResetAHRS函数结束

static void IMU_QuaternionToEuler(float *yaw, float *pitch, float *roll)  // 静态函数：由当前四元数解算出欧拉角（偏航/俯仰/横滚，单位°）
{                                               // IMU_QuaternionToEuler函数体开始
    float sin_pitch;                            // 临时变量：俯仰角的正弦值

    *yaw = -atan2f(                             // 用四元数转欧拉角公式计算偏航角yaw（取负号匹配小车坐标系方向），结果由弧度转成角度
        2.0f * imu_q1 * imu_q2 + 2.0f * imu_q0 * imu_q3,   // atan2的y参数：2(q1q2+q0q3)
        -2.0f * imu_q2 * imu_q2 - 2.0f * imu_q3 * imu_q3 + 1.0f  // atan2的x参数：1-2(q2²+q3²)
    ) * IMU_RAD_TO_DEG;                         // 乘以弧度转角度系数，得到以度为单位的yaw

    sin_pitch = -2.0f * imu_q1 * imu_q3 + 2.0f * imu_q0 * imu_q2;  // 计算俯仰角正弦值：2(q0q2-q1q3)
    sin_pitch = IMU_Clamp(sin_pitch, -1.0f, 1.0f);  // 限幅到[-1,1]，防止浮点误差导致asinf定义域溢出

    *pitch = -asinf(sin_pitch) * IMU_RAD_TO_DEG;    // 反正弦求俯仰角pitch（取负号匹配坐标系），转成角度

    *roll = atan2f(                             // 计算横滚角roll，结果由弧度转成角度
        2.0f * imu_q2 * imu_q3 + 2.0f * imu_q0 * imu_q1,   // atan2的y参数：2(q2q3+q0q1)
        -2.0f * imu_q1 * imu_q1 - 2.0f * imu_q2 * imu_q2 + 1.0f  // atan2的x参数：1-2(q1²+q2²)
    ) * IMU_RAD_TO_DEG;                         // 乘以弧度转角度系数，得到以度为单位的roll
}                                               // IMU_QuaternionToEuler函数结束

void IMU_ResetYaw(float yaw_deg)                // 对外接口：把当前偏航角重新设为指定值（常用于清零yaw）
{                                               // IMU_ResetYaw函数体开始
    float yaw;                                  // 临时变量：当前解算出的偏航角
    float pitch;                                // 临时变量：当前俯仰角（本函数不用，仅为调用所需）
    float roll;                                 // 临时变量：当前横滚角（本函数不用，仅为调用所需）

    IMU_QuaternionToEuler(&yaw, &pitch, &roll); // 先算出当前三个欧拉角
    imu_yaw_offset = IMU_Wrap180(yaw - yaw_deg);// 调整yaw零点偏移，使之后读出的yaw恰好等于yaw_deg（一般传0即清零）
}                                               // IMU_ResetYaw函数结束

static void IMU_AHRSUpdate(float gx, float gy, float gz,   // 静态函数：Mahony姿态融合算法核心，输入陀螺仪角速度（弧度/秒）、
                           float ax, float ay, float az,   // 加速度计三轴值（g）和采样周期dt，更新姿态四元数
                           float dt_s)
{                                               // IMU_AHRSUpdate函数体开始
    float norm;                                 // 归一化用的模长变量
    float vx;                                   // 由当前姿态估计出的重力方向X分量（机体坐标系）
    float vy;                                   // 估计重力方向Y分量
    float vz;                                   // 估计重力方向Z分量
    float ex;                                   // 加速度计实测重力与估计重力的误差X分量（叉积）
    float ey;                                   // 误差Y分量
    float ez;                                   // 误差Z分量
    float half_dt;                              // 半个采样周期，四元数微分方程积分时用

    float q0q0;                                 // 四元数乘积项q0*q0，预先算好避免重复乘法
    float q0q1;                                 // 乘积项q0*q1
    float q0q2;                                 // 乘积项q0*q2
    float q1q1;                                 // 乘积项q1*q1
    float q1q3;                                 // 乘积项q1*q3
    float q2q2;                                 // 乘积项q2*q2
    float q2q3;                                 // 乘积项q2*q3
    float q3q3;                                 // 乘积项q3*q3

    float tq0;                                  // 更新后的临时四元数分量q0
    float tq1;                                  // 临时四元数分量q1
    float tq2;                                  // 临时四元数分量q2
    float tq3;                                  // 临时四元数分量q3

    norm = ax * ax + ay * ay + az * az;         // 计算加速度向量的模长平方
    if (norm <= 0.000001f) {                    // 模长接近0说明加速度数据无效（如传感器异常），无法纠偏
        return;                                 // 直接返回，本次不做姿态更新，防止除以0
    }                                           // if判断结束

    norm = 1.0f / sqrtf(norm);                  // 模长平方开方后取倒数，得到归一化系数
    ax *= norm;                                 // 加速度X归一化为单位向量
    ay *= norm;                                 // 加速度Y归一化
    az *= norm;                                 // 加速度Z归一化

    half_dt = 0.5f * dt_s;                      // 计算半个采样周期，供误差积分和四元数微分使用

    q0q0 = imu_q0 * imu_q0;                     // 预计算q0*q0
    q0q1 = imu_q0 * imu_q1;                     // 预计算q0*q1
    q0q2 = imu_q0 * imu_q2;                     // 预计算q0*q2
    q1q1 = imu_q1 * imu_q1;                     // 预计算q1*q1
    q1q3 = imu_q1 * imu_q3;                     // 预计算q1*q3
    q2q2 = imu_q2 * imu_q2;                     // 预计算q2*q2
    q2q3 = imu_q2 * imu_q3;                     // 预计算q2*q3
    q3q3 = imu_q3 * imu_q3;                     // 预计算q3*q3

    vx = 2.0f * (q1q3 - q0q2);                  // 由当前姿态四元数推算机体重力方向X分量
    vy = 2.0f * (q0q1 + q2q3);                  // 推算重力方向Y分量
    vz = q0q0 - q1q1 - q2q2 + q3q3;             // 推算重力方向Z分量

    ex = ay * vz - az * vy;                     // 实测加速度与估计重力做叉积，得到姿态误差X分量
    ey = az * vx - ax * vz;                     // 误差Y分量
    ez = ax * vy - ay * vx;                     // 误差Z分量

    imu_ex_int += ex * IMU_MAHONY_KI * half_dt; // 误差乘以积分增益累加，形成X轴零偏积分补偿
    imu_ey_int += ey * IMU_MAHONY_KI * half_dt; // Y轴零偏积分补偿
    imu_ez_int += ez * IMU_MAHONY_KI * half_dt; // Z轴零偏积分补偿

    gx += IMU_MAHONY_KP * ex + imu_ex_int;      // 用比例+积分补偿修正陀螺仪X角速度（PI控制器）
    gy += IMU_MAHONY_KP * ey + imu_ey_int;      // 修正陀螺仪Y角速度
    gz += IMU_MAHONY_KP * ez + imu_ez_int;      // 修正陀螺仪Z角速度

    tq0 = imu_q0 + (-imu_q1 * gx - imu_q2 * gy - imu_q3 * gz) * half_dt;  // 四元数微分方程一阶积分，更新q0
    tq1 = imu_q1 + ( imu_q0 * gx + imu_q2 * gz - imu_q3 * gy) * half_dt;  // 更新q1
    tq2 = imu_q2 + ( imu_q0 * gy - imu_q1 * gz + imu_q3 * gx) * half_dt;  // 更新q2
    tq3 = imu_q3 + ( imu_q0 * gz + imu_q1 * gy - imu_q2 * gx) * half_dt;  // 更新q3

    norm = tq0 * tq0 + tq1 * tq1 + tq2 * tq2 + tq3 * tq3;  // 计算新四元数模长平方
    if (norm <= 0.000001f) {                    // 模长异常接近0（数值错误）
        IMU_ResetAHRS();                        // 复位姿态解算，恢复为单位四元数
        return;                                 // 本次更新结束
    }                                           // if判断结束

    norm = 1.0f / sqrtf(norm);                  // 求归一化系数

    imu_q0 = tq0 * norm;                        // 四元数q0归一化后写回（四元数必须保持模长为1）
    imu_q1 = tq1 * norm;                        // q1归一化写回
    imu_q2 = tq2 * norm;                        // q2归一化写回
    imu_q3 = tq3 * norm;                        // q3归一化写回
}                                               // IMU_AHRSUpdate函数结束

static int8_t IMU_ConfigAccelGyro(void)         // 静态函数：配置加速度计和陀螺仪的量程、采样率、滤波参数
{                                               // IMU_ConfigAccelGyro函数体开始
    int8_t rslt;                                // 存放驱动库返回值，BMI2_OK(0)表示成功
    struct bmi2_sens_config config[2];          // 两个传感器配置结构体：[0]加速度计，[1]陀螺仪

    config[0].type = BMI2_ACCEL;                // 第0项配置为加速度计
    config[1].type = BMI2_GYRO;                 // 第1项配置为陀螺仪

    rslt = bmi2_get_sensor_config(config, 2, &imu_dev);  // 先读出当前传感器配置（驱动要求先读后改）
    if (rslt != BMI2_OK) return rslt;           // 读配置失败则把错误码返回给上层

    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;  // 加速度计选性能优化滤波模式（平滑优先）
    config[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;          // 加速度计带宽参数：OSR2二倍过采样平均
    config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;          // 加速度计输出数据率200Hz
    config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;         // 加速度计量程±2g（平衡车倾角足够，分辨率最高）

    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;  // 陀螺仪选性能优化滤波模式
    config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;   // 陀螺仪噪声性能选优化模式（低噪声）
    config[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;          // 陀螺仪带宽参数：OSR2模式
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;          // 陀螺仪输出数据率200Hz
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;       // 陀螺仪量程±2000°/s
    config[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;     // OIS光学防抖通道量程±2000°/s（驱动要求配套设置）

    return bmi2_set_sensor_config(config, 2, &imu_dev);  // 把修改后的配置写回芯片并返回结果
}                                               // IMU_ConfigAccelGyro函数结束

int8_t IMU_Init(void)                           // 对外接口：初始化IMU（SPI、芯片、传感器配置并使能），返回0成功
{                                               // IMU_Init函数体开始
    int8_t rslt;                                // 存放驱动库返回值
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };  // 要使能的传感器列表：加速度计和陀螺仪

    memset(&imu_dev, 0, sizeof(imu_dev));       // 把设备结构体全部清零，避免残留垃圾值

    BMI270_SPI_Init();                          // 初始化SPI外设和片选引脚（连接BMI270的硬件层）

    (void)BMI270_SPI_ReadID();                  // 读一次芯片ID唤醒/检测SPI通信（结果丢弃，BMI270上电后首次读取需dummy）

    imu_dev.intf = BMI2_SPI_INTF;               // 告诉驱动库使用SPI接口（而非I2C）
    imu_dev.read = BMI270_SPI_Read;             // 注册SPI读寄存器函数指针给驱动库
    imu_dev.write = BMI270_SPI_Write;           // 注册SPI写寄存器函数指针给驱动库
    imu_dev.delay_us = BMI270_SPI_DelayUs;      // 注册微秒延时函数指针给驱动库
    imu_dev.read_write_len = IMU_READ_WRITE_LEN;// 设置SPI单次读写最大长度46字节
    imu_dev.config_file_ptr = NULL;             // 不使用外部配置文件（BMI270配置流内置于驱动）
    imu_dev.intf_ptr = NULL;                    // 接口私有指针为空（SPI只有一个实例，无需区分）

    rslt = bmi270_init(&imu_dev);               // 调用官方驱动初始化BMI270（软复位、加载配置流、校验芯片）
    if (rslt != BMI2_OK) return rslt;           // 初始化失败直接返回错误码

    rslt = IMU_ConfigAccelGyro();               // 配置加速度计/陀螺仪的量程、采样率、滤波
    if (rslt != BMI2_OK) return rslt;           // 配置失败返回错误码

    rslt = bmi2_sensor_enable(sensor_list, 2, &imu_dev);  // 使能加速度计和陀螺仪，芯片开始输出数据
    if (rslt == BMI2_OK) {                      // 使能成功
        imu_is_init = 1;                        // 置初始化成功标志，允许后续读数
        IMU_ResetAHRS();                        // 复位姿态解算状态，从单位四元数开始
    }                                           // if判断结束

    return rslt;                                // 返回初始化结果
}                                               // IMU_Init函数结束

int8_t IMU_ReadRaw(IMU_RawData *data)           // 对外接口：读取三轴加速度+三轴陀螺仪的原始ADC整数值
{                                               // IMU_ReadRaw函数体开始
    int8_t rslt;                                // 存放驱动库返回值
    struct bmi2_sens_data sensor_data = { { 0 } };  // 驱动库传感器数据结构体，清零备用

    if ((imu_is_init == 0) || (data == 0)) {    // 未初始化或输出指针为空，属于非法调用
        return BMI2_E_NULL_PTR;                 // 返回空指针错误码
    }                                           // if判断结束

    rslt = bmi2_get_sensor_data(&sensor_data, &imu_dev);  // 从BMI270读取最新的加速度和陀螺仪数据
    if (rslt != BMI2_OK) return rslt;           // 读取失败返回错误码

    if (((sensor_data.status & BMI2_DRDY_ACC) == 0) ||   // 检查状态位：加速度数据是否就绪
        ((sensor_data.status & BMI2_DRDY_GYR) == 0)) {   // 检查状态位：陀螺仪数据是否就绪
        return IMU_NOT_READY;                   // 任一新数据未就绪，返回"数据未准备好"
    }                                           // if判断结束

    data->acc_x = sensor_data.acc.x;            // 取出加速度X轴原始值（LSB整数）
    data->acc_y = sensor_data.acc.y;            // 取出加速度Y轴原始值
    data->acc_z = sensor_data.acc.z;            // 取出加速度Z轴原始值
    data->gyr_x = sensor_data.gyr.x;            // 取出陀螺仪X轴原始值
    data->gyr_y = sensor_data.gyr.y;            // 取出陀螺仪Y轴原始值
    data->gyr_z = sensor_data.gyr.z;            // 取出陀螺仪Z轴原始值

    return IMU_OK;                              // 读取成功
}                                               // IMU_ReadRaw函数结束

static int8_t IMU_ReadDataNoOffset(IMU_Data *data)  // 静态函数：读原始值并换算成物理单位（g和°/s），但不减零偏，并计算加速度姿态角
{                                               // IMU_ReadDataNoOffset函数体开始
    IMU_RawData raw;                            // 存放原始整数值的结构体
    int8_t rslt;                                // 存放返回值

    if (data == 0) {                            // 输出指针为空属非法调用
        return BMI2_E_NULL_PTR;                 // 返回空指针错误码
    }                                           // if判断结束

    rslt = IMU_ReadRaw(&raw);                   // 先读取原始数据
    if (rslt != IMU_OK) {                       // 读取失败
        return rslt;                            // 把错误码透传给上层
    }                                           // if判断结束

    data->acc_x_g = raw.acc_x / IMU_ACC_LSB_PER_G;  // 加速度X原始值除以灵敏度16384LSB/g，换算成g单位
    data->acc_y_g = raw.acc_y / IMU_ACC_LSB_PER_G;  // 加速度Y换算成g
    data->acc_z_g = raw.acc_z / IMU_ACC_LSB_PER_G;  // 加速度Z换算成g

    data->gyr_x_dps = raw.gyr_x / IMU_GYR_LSB_PER_DPS;  // 陀螺仪X原始值除以16.384LSB/(°/s)，换算成度/秒
    data->gyr_y_dps = raw.gyr_y / IMU_GYR_LSB_PER_DPS;  // 陀螺仪Y换算成度/秒
    data->gyr_z_dps = raw.gyr_z / IMU_GYR_LSB_PER_DPS;  // 陀螺仪Z换算成度/秒

    data->roll_acc = atan2f(data->acc_y_g, data->acc_z_g) * IMU_RAD_TO_DEG;  // 仅用加速度计估算横滚角（静止时准，运动时受干扰）
    data->pitch_acc = atan2f(                   // 仅用加速度计估算俯仰角
        -data->acc_x_g,                         // atan2的y参数：负的X轴加速度
        sqrtf(data->acc_y_g * data->acc_y_g +   // atan2的x参数：Y、Z轴加速度合成向量长度
              data->acc_z_g * data->acc_z_g)
    ) * IMU_RAD_TO_DEG;                         // 结果转成角度

    return IMU_OK;                              // 读取换算成功
}                                               // IMU_ReadDataNoOffset函数结束

int8_t IMU_ReadData(IMU_Data *data)             // 对外接口：以默认周期10ms读取并解算完整姿态数据
{                                               // IMU_ReadData函数体开始
    return IMU_ReadDataDt(data, IMU_DEFAULT_DT_S);  // 等价于调用带dt版本，传入默认0.01s
}                                               // IMU_ReadData函数结束

int8_t IMU_ReadDataDt(IMU_Data *data, float dt_s)  // 对外接口：按指定采样周期读取数据并做Mahony融合，输出欧拉角
{                                               // IMU_ReadDataDt函数体开始
    int8_t rslt;                                // 存放返回值
    float yaw;                                  // 融合后的偏航角（未减偏移）
    float pitch;                                // 融合后的俯仰角
    float roll;                                 // 融合后的横滚角
    float gx;                                   // X轴角速度（弧度/秒），供四元数积分
    float gy;                                   // Y轴角速度（弧度/秒）
    float gz;                                   // Z轴角速度（弧度/秒）

    if (dt_s <= 0.0f) {                         // 传入的周期非法（<=0）
        dt_s = IMU_DEFAULT_DT_S;                // 改用默认周期0.01s，防止积分出错
    }                                           // if判断结束

    rslt = IMU_ReadDataNoOffset(data);          // 读取并换算物理量（此时未减零偏）
    if (rslt != IMU_OK) {                       // 读取失败
        return rslt;                            // 返回错误码
    }                                           // if判断结束

    data->gyr_x_dps -= imu_gyr_x_offset;        // 陀螺仪X减去校准得到的零偏
    data->gyr_y_dps -= imu_gyr_y_offset;        // 陀螺仪Y减去零偏
    data->gyr_z_dps -= imu_gyr_z_offset;        // 陀螺仪Z减去零偏

    /* 静止时在线跟踪零偏：陀螺仪零偏会随温度缓慢变化，
       静止期间把残余角速度按很小比例回灌到零偏里，抑制长期漂移；
       电机转动/车身振动时加速度模长偏离1g，自动停止跟踪 */
    if (fabsf(data->gyr_x_dps) < IMU_STILL_GYRO_DPS &&   // X轴角速度很小
        fabsf(data->gyr_y_dps) < IMU_STILL_GYRO_DPS &&   // 且Y轴角速度很小
        fabsf(data->gyr_z_dps) < IMU_STILL_GYRO_DPS) {   // 且Z轴角速度很小，才可能处于静止
        float acc_norm = sqrtf(data->acc_x_g * data->acc_x_g +   // 计算加速度向量模长
                               data->acc_y_g * data->acc_y_g +
                               data->acc_z_g * data->acc_z_g);
        if (fabsf(acc_norm - 1.0f) < IMU_STILL_ACC_G) {  // 模长接近1g说明无运动无振动，确认静止
            imu_gyr_x_offset += IMU_BIAS_ALPHA * data->gyr_x_dps;  // 把X残余角速度的一小部分回灌进零偏估计
            imu_gyr_y_offset += IMU_BIAS_ALPHA * data->gyr_y_dps;  // Y轴零偏在线跟踪
            imu_gyr_z_offset += IMU_BIAS_ALPHA * data->gyr_z_dps;  // Z轴零偏在线跟踪
        }                                       // 内层if（确认静止）结束
    }                                           // 外层if（静止判定）结束

    /* 零速死区：微小残余零偏/噪声不参与积分，减缓yaw漂移 */
    if (fabsf(data->gyr_x_dps) < IMU_GYRO_DEADBAND_DPS) data->gyr_x_dps = 0.0f;  // X角速度小于死区阈值则清零
    if (fabsf(data->gyr_y_dps) < IMU_GYRO_DEADBAND_DPS) data->gyr_y_dps = 0.0f;  // Y角速度小于死区则清零
    if (fabsf(data->gyr_z_dps) < IMU_GYRO_DEADBAND_DPS) data->gyr_z_dps = 0.0f;  // Z角速度小于死区则清零

    gx = data->gyr_x_dps * IMU_DEG_TO_RAD;      // X角速度由度/秒转成弧度/秒（四元数积分要求弧度）
    gy = data->gyr_y_dps * IMU_DEG_TO_RAD;      // Y角速度转弧度
    gz = data->gyr_z_dps * IMU_DEG_TO_RAD;      // Z角速度转弧度

    IMU_AHRSUpdate(gx, gy, gz,                  // 执行一次Mahony姿态融合：陀螺仪积分+加速度计纠偏
                   data->acc_x_g, data->acc_y_g, data->acc_z_g,
                   dt_s);

    IMU_QuaternionToEuler(&yaw, &pitch, &roll); // 把更新后的四元数转成欧拉角

    data->yaw = IMU_Wrap180(yaw - imu_yaw_offset);   // 减去yaw零点偏移并归一化到±180°，作为输出偏航角
    data->pitch = pitch - imu_pitch_offset;     // 减去俯仰角零点偏移
    data->roll = roll - imu_roll_offset;        // 减去横滚角零点偏移

    data->roll_acc -= imu_roll_offset;          // 纯加速度计估算的横滚角同样减零点偏移，保持基准一致
    data->pitch_acc -= imu_pitch_offset;        // 纯加速度计估算的俯仰角同样减零点偏移

    return IMU_OK;                              // 姿态解算完成，返回成功
}                                               // IMU_ReadDataDt函数结束

int8_t IMU_Calibrate(uint16_t samples)          // 对外接口：静止状态下校准，多次采样求平均得到陀螺零偏和姿态零点
{                                               // IMU_Calibrate函数体开始
    IMU_Data data;                              // 存放每次采样数据的结构体
    uint16_t i;                                 // 循环计数器
    uint16_t ok_count;                          // 有效采样计数（读取成功的次数）
    uint8_t attempt;                            // 重试次数计数器

    float sum_gx;                               // 陀螺仪X角速度累加和
    float sum_gy;                               // 陀螺仪Y角速度累加和
    float sum_gz;                               // 陀螺仪Z角速度累加和
    float sum_roll;                             // 加速度计横滚角累加和
    float sum_pitch;                            // 加速度计俯仰角累加和

    float min_gx, max_gx;                       // 采样期间X角速度的最小/最大值（求极差判静止用）
    float min_gy, max_gy;                       // Y角速度最小/最大值
    float min_gz, max_gz;                       // Z角速度最小/最大值

    if (samples == 0) {                         // 传入采样数为0
        samples = 200;                          // 使用默认200次采样（约1秒，平均效果好）
    }                                           // if判断结束

    for (attempt = 0; attempt < IMU_CAL_MAX_RETRY; attempt++) {  // 最多重试IMU_CAL_MAX_RETRY次
        sum_gx = 0.0f;                          // 每轮重试前清累加和
        sum_gy = 0.0f;
        sum_gz = 0.0f;
        sum_roll = 0.0f;
        sum_pitch = 0.0f;
        ok_count = 0;                           // 清有效采样计数
        min_gx = min_gy = min_gz = 1e9f;        // 最小值初始为很大的数
        max_gx = max_gy = max_gz = -1e9f;       // 最大值初始为很小的数

        for (i = 0; i < samples; i++) {         // 循环采样samples次（校准时小车必须保持静止）
            if (IMU_ReadDataNoOffset(&data) == IMU_OK) {  // 读取一次未减零偏的数据，成功才处理
                if (i >= IMU_CAL_SKIP_SAMPLES) {   // 头部几个样本传感器未稳定，丢弃不参与统计
                    sum_gx += data.gyr_x_dps;   // 累加X角速度（静止时的读数就是零偏）
                    sum_gy += data.gyr_y_dps;   // 累加Y角速度
                    sum_gz += data.gyr_z_dps;   // 累加Z角速度
                    sum_roll += data.roll_acc;  // 累加加速度计横滚角（作为横滚零点）
                    sum_pitch += data.pitch_acc;// 累加加速度计俯仰角（作为俯仰零点）
                    ok_count++;                 // 有效采样数加1

                    if (data.gyr_x_dps < min_gx) min_gx = data.gyr_x_dps;  // 更新X最小值
                    if (data.gyr_x_dps > max_gx) max_gx = data.gyr_x_dps;  // 更新X最大值
                    if (data.gyr_y_dps < min_gy) min_gy = data.gyr_y_dps;  // 更新Y最小值
                    if (data.gyr_y_dps > max_gy) max_gy = data.gyr_y_dps;  // 更新Y最大值
                    if (data.gyr_z_dps < min_gz) min_gz = data.gyr_z_dps;  // 更新Z最小值
                    if (data.gyr_z_dps > max_gz) max_gz = data.gyr_z_dps;  // 更新Z最大值
                }                               // if 丢弃头部样本判断结束
            }                                   // if读取成功判断结束

            imu_dev.delay_us(IMU_CAL_DELAY_US, imu_dev.intf_ptr);  // 延时5ms再采下一次，等待传感器输出新数据
        }                                       // for采样循环结束

        if (ok_count == 0) {                    // 一次都没读成功，传感器异常
            return IMU_NOT_READY;               // 返回未就绪错误（重试也无意义，通信本身有问题）
        }                                       // if判断结束

        if ((max_gx - min_gx) < IMU_CAL_STILL_RANGE_DPS &&   // X轴角速度极差很小
            (max_gy - min_gy) < IMU_CAL_STILL_RANGE_DPS &&   // Y轴极差很小
            (max_gz - min_gz) < IMU_CAL_STILL_RANGE_DPS) {   // Z轴极差也很小：确认全程静止，校准有效
            imu_gyr_x_offset = sum_gx / ok_count;   // 平均值作为陀螺仪X零偏
            imu_gyr_y_offset = sum_gy / ok_count;   // 平均值作为陀螺仪Y零偏
            imu_gyr_z_offset = sum_gz / ok_count;   // 平均值作为陀螺仪Z零偏

            imu_roll_offset = sum_roll / ok_count;  // 平均横滚角作为横滚零点（当前姿态视为0°）
            imu_pitch_offset = sum_pitch / ok_count;// 平均俯仰角作为俯仰零点
            imu_yaw_offset = 0.0f;              // yaw零点清零（yaw无法靠加速度计校准，只能从0开始积分）

            IMU_ResetAHRS();                    // 复位姿态解算，以校准后的姿态重新开始融合

            return IMU_OK;                      // 校准成功
        }                                       // if 静止检测通过结束

        /* 检测到移动：本轮数据作废，等一会再重试 */
        imu_dev.delay_us(IMU_CAL_RETRY_DELAY_US, imu_dev.intf_ptr);  // 等待500ms，给放下小车的时间
    }                                           // for重试循环结束

    /* 多次重试都检测到移动：用内置的兜底零偏，姿态零点按水平处理，
       返回 IMU_CAL_FALLBACK 告知调用方"校准失败但已用兜底值，系统可继续跑" */
    imu_gyr_x_offset = IMU_GYRO_OFFSET_FALLBACK_X;  // X轴使用兜底零偏
    imu_gyr_y_offset = IMU_GYRO_OFFSET_FALLBACK_Y;  // Y轴使用兜底零偏
    imu_gyr_z_offset = IMU_GYRO_OFFSET_FALLBACK_Z;  // Z轴使用兜底零偏

    imu_roll_offset = 0.0f;                     // 横滚零点按0处理（默认车放水平）
    imu_pitch_offset = 0.0f;                    // 俯仰零点按0处理
    imu_yaw_offset = 0.0f;                      // yaw零点清零

    IMU_ResetAHRS();                            // 复位姿态解算

    return IMU_CAL_FALLBACK;                    // 返回"使用了兜底零偏"的状态码
}                                               // IMU_Calibrate函数结束
uint8_t IMU_DebugGetChipId(void)                // 对外接口：调试用，返回BMI270的芯片ID（正常应为0x24）
{                                               // IMU_DebugGetChipId函数体开始
    return imu_dev.chip_id;                     // 返回驱动初始化时读到的芯片ID
}                                               // IMU_DebugGetChipId函数结束
