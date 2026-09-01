#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

// NFP1315-157B I2C OLED 引脚（按实物可调整）
#define DISPLAY_SDA_PIN 16
#define DISPLAY_SCL_PIN 17

// 初始化屏幕
void displayInit();

// 渲染比赛信息
// matchStarted : true=已收到 START（比赛中），false=等待开始
// vibrationActive : 当前锁存振动编号，-1 表示无振动
// laserState : 4 路激光当前状态（1=当前被遮挡）
// elapsedSeconds : 比赛已进行秒数（matchStarted=false 时可为 0）
void displayRender(bool matchStarted, int vibrationActive,
                   const bool laserState[4], unsigned long elapsedSeconds);

#endif // DISPLAY_H
