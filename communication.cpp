
#include <Arduino.h>

static int count = 0;

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 41, 42);
}

void loop()
{
    // 发送数据包 — 使用 Print::printf 直接格式化，避免 String 拼接的堆分配
    Serial2.printf("MSG:%d|Hello ESP32-S3\r\n", count);
    Serial.printf("已发送：MSG:%d|Hello ESP32-S3\r\n", count);

    // 接收应答 — 使用 Stream::available / Stream::readStringUntil
    if (Serial2.available() > 0)
    {
        String recvAck = Serial2.readStringUntil('\n');
        Serial.printf("收到应答：%s\n", recvAck.c_str());
    }

    count++;
    delay(1000);
}