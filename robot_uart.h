#ifndef __ROBOT_UART_H__
#define __ROBOT_UART_H__

#define SERIAL  "/dev/ttyUSB0"          /* 串口设备文件 */ 

typedef int RSHandle;

/* 建立串口连接 */
bool setupConnection(RSHandle * handle);

/* 关闭串口连接 */
bool closeConnection(RSHandle * handle);

/* 设置线速度，单位：毫米/s，前进为正，后退为负 */
bool setLineVelocity(RSHandle handle, float lineVelo);

/* 设置角速度，单位：毫弧度/s，逆时针为正，顺时针为负 */
bool setAngleVelocity(RSHandle handle, float AngleVelo);

/* 同时设置线速度和角速度 */
bool setLineAngleVelocity(RSHandle handle, float lineVelo, float AngleVelo);

/* 获取码盘积分位置，返回左轮和右轮码盘积分位置 */
bool getRobotEncoderPos(RSHandle handle, short &leftPos, short &rightPos);

/* 获取红外测距结果，返回bool型数组，数组长度大于等于5，true表示有障碍物
 * 方向从infraRed[0] ~ infraRed[4]依次为: 右、右前、正前、左、左前 */
bool getRobotInfrared(RSHandle handle, bool *infraRed);

/* 获取电量，返回一个unsigned char数值，代表还有百分之几的电量 */
/*
 * 由于银星目前版本调试程序没有上报电量信息
 * 所以该函数暂不可用
 */
bool getRobotBattery(RSHandle handle, unsigned char &percentage);

#endif
