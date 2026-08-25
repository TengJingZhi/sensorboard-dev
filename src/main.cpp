#include <Arduino.h>
#include <string.h>

// ==================== 可调参数 ====================
// 振动开关屏蔽时间：该时间内锁存最先触发的振动位并持续上报为 1
#ifndef VIBRATION_BLOCK_MS
#define VIBRATION_BLOCK_MS 2000UL
#endif

// 心跳发送周期：数据无变化时也按此周期发送一次
#ifndef HEARTBEAT_MS
#define HEARTBEAT_MS 200UL
#endif

// 最小发送间隔：防止快速抖动导致总线拥塞
#ifndef MIN_SEND_INTERVAL_MS
#define MIN_SEND_INTERVAL_MS 10UL
#endif

// 激光遮挡判定阈值（analogRead，ESP32-S3 为 0~4095）
// 现场根据实际高电平调整；也可通过 platformio.ini 的 build_flags 覆盖
#ifndef LASER_THRESHOLD
#define LASER_THRESHOLD 2048
#endif

// 激光被遮挡时输出高电平：1 = 高电平表示遮挡，0 = 低电平表示遮挡
#ifndef LASER_ACTIVE_HIGH
#define LASER_ACTIVE_HIGH 1
#endif

// ==================== 引脚定义 ====================
const int viberatorPins[] = {8, 9, 10, 11, 12, 13, 14};
#define VIBERATION_COUNT 7

const int laserPins[] = {4, 5, 6, 7};
#define LASER_COUNT 4

const int laserOutputPin = 15;

const int ReceiveMsgPin = 41;
const int SendMsgPin = 42;

// ==================== UART 协议（与主板 esp-cam 一致） ====================
#define FRAME_HEADER0 0xAA
#define FRAME_HEADER1 0x55
#define FRAME_TAIL 0x0D
#define UART_MSG_SENSOR_DATA 0x01
#define SENSOR_PAYLOAD_LEN (VIBERATION_COUNT + LASER_COUNT) // 7 + 4 = 11

// ==================== 从板状态 ====================
// 激光累积触发历史：一旦触发永久保持 1
bool laserAcc[LASER_COUNT] = {false};

// 当前锁存的振动开关编号，-1 表示无振动事件
int vibrationActive = -1;
unsigned long vibrationBlockUntil = 0;

// 上次实际发送的 payload，用于“变化触发”
uint8_t lastSentPayload[SENSOR_PAYLOAD_LEN] = {0};
unsigned long lastSendTime = 0;

// ==================== 发送函数 ====================
void sendSensorFrame(const uint8_t payload[SENSOR_PAYLOAD_LEN])
{
    uint8_t checksum = UART_MSG_SENSOR_DATA ^ SENSOR_PAYLOAD_LEN;
    for (int i = 0; i < SENSOR_PAYLOAD_LEN; i++)
    {
        checksum ^= payload[i];
    }

    Serial2.write(FRAME_HEADER0);
    Serial2.write(FRAME_HEADER1);
    Serial2.write(UART_MSG_SENSOR_DATA);
    Serial2.write(SENSOR_PAYLOAD_LEN);
    for (int i = 0; i < SENSOR_PAYLOAD_LEN; i++)
    {
        Serial2.write(payload[i]);
    }
    Serial2.write(checksum);
    Serial2.write(FRAME_TAIL);

    Serial.write(FRAME_HEADER0);
    Serial.write(FRAME_HEADER1);
    Serial.write(UART_MSG_SENSOR_DATA);
    Serial.write(SENSOR_PAYLOAD_LEN);
    for (int i = 0; i < SENSOR_PAYLOAD_LEN; i++)
    {
        Serial.write(payload[i]);
    }
    Serial.write(checksum);
    Serial.write(FRAME_TAIL);
}

// ==================== 传感器状态更新 ====================
void updateVibration()
{
    unsigned long now = millis();

    // 屏蔽期结束：清除当前锁存，恢复检测
    if (vibrationActive >= 0 && (long)(now - vibrationBlockUntil) >= 0)
    {
        vibrationActive = -1;
    }

    // 无锁存时，锁存最先触发的一路振动
    if (vibrationActive < 0)
    {
        for (int i = 0; i < VIBERATION_COUNT; i++)
        {
            if (digitalRead(viberatorPins[i]) == HIGH)
            {
                vibrationActive = i;
                vibrationBlockUntil = now + VIBRATION_BLOCK_MS;
                Serial.printf("[SLAVE] 振动触发并锁存: Switch No.%d, 屏蔽 %lu ms\n",
                              i + 1, (unsigned long)VIBRATION_BLOCK_MS);
                break;
            }
        }
    }
}

void updateLasers()
{
    for (int i = 0; i < LASER_COUNT; i++)
    {
        int value = analogRead(laserPins[i]);
        bool triggered;
        if (LASER_ACTIVE_HIGH)
        {
            triggered = (value > LASER_THRESHOLD);
        }
        else
        {
            triggered = (value < LASER_THRESHOLD);
        }

        // 累积触发历史：一旦触发保持为 1
        if (triggered)
        {
            laserAcc[i] = true;
        }
    }
}

// ==================== 组帧 ====================
void buildPayload(uint8_t payload[SENSOR_PAYLOAD_LEN])
{
    memset(payload, 0, SENSOR_PAYLOAD_LEN);

    // 振动位：只保留当前锁存的那一路
    if (vibrationActive >= 0 && vibrationActive < VIBERATION_COUNT)
    {
        payload[vibrationActive] = 1;
    }

    // 激光位：使用累积触发历史
    for (int i = 0; i < LASER_COUNT; i++)
    {
        payload[VIBERATION_COUNT + i] = laserAcc[i] ? 1 : 0;
    }
}

// ==================== 发送调度：变化触发 + 周期心跳 ====================
void sendIfNeeded(const uint8_t payload[SENSOR_PAYLOAD_LEN])
{
    unsigned long now = millis();
    bool changed = memcmp(payload, lastSentPayload, SENSOR_PAYLOAD_LEN) != 0;

    // 最小发送间隔保护
    if (now - lastSendTime < MIN_SEND_INTERVAL_MS)
    {
        return;
    }

    // 变化立即发；无变化时按心跳发
    if (changed || (now - lastSendTime >= HEARTBEAT_MS))
    {
        sendSensorFrame(payload);
        memcpy(lastSentPayload, payload, SENSOR_PAYLOAD_LEN);
        lastSendTime = now;

        if (changed)
        {
            Serial.printf("[SLAVE] 变化发送: 振动锁存=%d, 激光累积=[%d,%d,%d,%d]\n",
                          vibrationActive,
                          (int)laserAcc[0], (int)laserAcc[1],
                          (int)laserAcc[2], (int)laserAcc[3]);
        }
    }
}

// ==================== 主入口 ====================
void setup()
{
    Serial.begin(115200); // 调试串口
    Serial2.begin(115200, SERIAL_8N1, ReceiveMsgPin, SendMsgPin); // 与主板通信

    // 初始化振动传感器引脚
    for (int i = 0; i < VIBERATION_COUNT; i++)
    {
        pinMode(viberatorPins[i], INPUT_PULLDOWN);
    }

    // 初始化激光引脚
    pinMode(laserOutputPin, OUTPUT);
    digitalWrite(laserOutputPin, HIGH); // 开启激光发射器（若不亮改为 LOW）
    for (int i = 0; i < LASER_COUNT; i++)
    {
        pinMode(laserPins[i], INPUT);
    }

    lastSendTime = millis();

    Serial.println("========================================");
    Serial.println("  从板启动：11字节传感器数据上报");
    Serial.println("  payload = isCrashed[7] + isCovered[4]");
    Serial.println("========================================");
}

void loop()
{
    updateVibration();
    updateLasers();

    uint8_t payload[SENSOR_PAYLOAD_LEN];
    buildPayload(payload);
    sendIfNeeded(payload);

    delay(1); // 轻微让出 CPU；硬件已 RC 消抖，无需长延时
}
