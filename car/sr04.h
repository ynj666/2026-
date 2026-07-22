#ifndef SR04_H  // 头文件保护：防止本头文件被重复包含
#define SR04_H  // 定义保护宏，再次包含时跳过全部内容

#include <stdint.h>  // 包含标准整数类型头文件，提供uint16_t等类型

#define SR04_TIMEOUT_CM  ((uint16_t)0xFFFF)  // 测距超时/无效时的返回值：0xFFFF，表示未测到有效距离

void SR04_Init(void);  // 声明：SR04超声波模块初始化（配置ECHO捕获定时器和TRIG引脚）
uint16_t SR04_ReadCm(void);  // 声明：执行一次测距，返回距离（厘米），超时返回SR04_TIMEOUT_CM

#endif  // 头文件保护结束，对应开头的#ifndef SR04_H