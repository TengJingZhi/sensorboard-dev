#include "display.h"

#include <stdio.h>

#include <Wire.h>
#include <U8g2lib.h>

// NFP1315-157B 按 SSD1315 的 128x64 I2C OLED 驱动
U8G2_SSD1315_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE,   // 无 RESET 引脚
    DISPLAY_SCL_PIN, // SCL
    DISPLAY_SDA_PIN  // SDA
);

void displayInit()
{
    u8g2.begin();
    u8g2.clear();

    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 12, "Sensor Board");
        u8g2.drawStr(0, 28, "Display Ready");
    } while (u8g2.nextPage());
}

void displayRender(bool matchStarted, int vibrationActive,
                   const bool laserState[4], unsigned long elapsedSeconds)
{
    char statusLine[28];
    char vibLine[16];
    char lsrLine[8];

    if (matchStarted)
    {
        snprintf(statusLine, sizeof(statusLine), "Status: RUN %lus",
                 (unsigned long)elapsedSeconds);
    }
    else
    {
        snprintf(statusLine, sizeof(statusLine), "Status: WAIT");
    }

    // 振动：当前锁存一路为 1，其余为 0
    for (int i = 0; i < 7; i++)
    {
        vibLine[i] = (i == vibrationActive) ? '1' : '0';
    }
    vibLine[7] = '\0';

    // 激光：当前状态（1=当前被遮挡）
    for (int i = 0; i < 4; i++)
    {
        lsrLine[i] = laserState[i] ? '1' : '0';
    }
    lsrLine[4] = '\0';

    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_6x10_tf);

        u8g2.drawStr(0, 12, statusLine);
        u8g2.drawStr(0, 26, "Vib:");
        u8g2.drawStr(28, 26, vibLine);
        u8g2.drawStr(0, 40, "Lsr:");
        u8g2.drawStr(28, 40, lsrLine);
        u8g2.drawStr(0, 54, "Auto refresh");
    } while (u8g2.nextPage());
}
