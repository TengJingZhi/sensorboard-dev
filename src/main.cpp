#include <Arduino.h>
#include <string.h>

#include "display.h"

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
#define UART_MSG_START 0x02       // 主板→从板：请求开始（三次握手第1步）
#define UART_MSG_START_ACK 0x04   // 从板→主板：确认就绪（三次握手第2步）
#define UART_MSG_START_CONFIRM 0x05 // 主板→从板：最终确认（三次握手第3步）
#define SENSOR_PAYLOAD_LEN (VIBERATION_COUNT + LASER_COUNT) // 7 + 4 = 11

// ==================== 从板状态 ====================
// 激光累积触发历史：一旦触发永久保持 1
bool laserAcc[LASER_COUNT] = {false};

// 激光当前遮挡状态：每轮采样刷新，仅供显示
bool laserState[LASER_COUNT] = {false};

// 当前锁存的振动开关编号，-1 表示无振动事件
int vibrationActive = -1;
unsigned long vibrationBlockUntil = 0;

// 上次实际发送的 payload，用于“变化触发”
uint8_t lastSentPayload[SENSOR_PAYLOAD_LEN] = {0};
unsigned long lastSendTime = 0;

// 流程开始标志：收到主板 START_CONFIRM 指令后才开始正式上报
bool flowStarted = false;
unsigned long matchStartedAt = 0;

// 三次握手状态机
typedef enum
{
    HS_IDLE = 0,    // 等待主板 START_REQ (0x02)
    HS_ACK_SENT,    // 已回复 ACK (0x04)，等待 START_CONFIRM (0x05)
    HS_DONE         // 握手完成，比赛中
} HandshakeState;

HandshakeState handshakeState = HS_IDLE;

// ==================== 显示状态 ====================
// 用于“变化触发 + 周期刷新”的显示快照
bool lastFlowStarted = false;
int lastVibrationActive = -1;
bool lastLaserState[LASER_COUNT] = {false};
bool displayDirty = true; // 上电后先刷一次
unsigned long lastDisplayTime = 0;
#define DISPLAY_REFRESH_MS 500UL

// ==================== 接收状态机（解析主板下发的指令） ====================
typedef enum
{
    RX_STATE_WAIT_HEADER0 = 0,
    RX_STATE_WAIT_HEADER1,
    RX_STATE_READ_MSG_TYPE,
    RX_STATE_READ_PAYLOAD_LEN,
    RX_STATE_READ_PAYLOAD,
    RX_STATE_READ_CHECKSUM,
    RX_STATE_READ_TAIL
} RxState;

static RxState rxState = RX_STATE_WAIT_HEADER0;
static uint8_t rxMsgType = 0;
static uint8_t rxPayloadLen = 0;
static uint8_t rxPayload[16];
static size_t rxPayloadIndex = 0;
static uint8_t rxChecksum = 0;

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

    // 调试串口可视化：以十六进制文本打印整帧，便于在串口监视器观察
    Serial.print("[TX] ");
    Serial.printf("%02X ", FRAME_HEADER0);
    Serial.printf("%02X ", FRAME_HEADER1);
    Serial.printf("%02X ", UART_MSG_SENSOR_DATA);
    Serial.printf("%02X ", SENSOR_PAYLOAD_LEN);
    for (int i = 0; i < SENSOR_PAYLOAD_LEN; i++)
    {
        Serial.printf("%02X ", payload[i]);
    }
    Serial.printf("%02X ", checksum);
    Serial.printf("%02X\n", FRAME_TAIL);
}

// ==================== 控制帧发送（0-payload 握手帧） ====================
void sendControlFrame(uint8_t msgType)
{
    uint8_t checksum = msgType ^ 0; // payload 长度为 0

    Serial2.write(FRAME_HEADER0);
    Serial2.write(FRAME_HEADER1);
    Serial2.write(msgType);
    Serial2.write(0); // payload len = 0
    Serial2.write(checksum);
    Serial2.write(FRAME_TAIL);

    // 调试串口可视化
    Serial.print("[TX] ");
    Serial.printf("%02X ", FRAME_HEADER0);
    Serial.printf("%02X ", FRAME_HEADER1);
    Serial.printf("%02X ", msgType);
    Serial.printf("%02X ", 0);
    Serial.printf("%02X ", checksum);
    Serial.printf("%02X\n", FRAME_TAIL);
}

// ==================== 比赛状态重置（用于比赛重开） ====================
void resetMatchState()
{
    flowStarted = false;
    handshakeState = HS_IDLE;
    memset(laserAcc, 0, sizeof(laserAcc));
    memset(laserState, 0, sizeof(laserState));
    vibrationActive = -1;
    vibrationBlockUntil = 0;
    memset(lastSentPayload, 0, sizeof(lastSentPayload));
    displayDirty = true;
    Serial.println("[SLAVE] 比赛状态已重置");
}

// ==================== 接收函数 ====================
static void resetRxState()
{
    rxState = RX_STATE_WAIT_HEADER0;
    rxPayloadIndex = 0;
    rxChecksum = 0;
}

static void handleRxFrame(uint8_t msgType, const uint8_t *payload, uint8_t len)
{
    if (msgType == UART_MSG_START && len == 0)
    {
        // 三次握手第1步：收到主板 START_REQ (0x02)
        // 若比赛已在进行（重开场景），先重置状态
        if (handshakeState == HS_DONE)
        {
            resetMatchState();
        }
        // 回复 ACK (0x04)
        sendControlFrame(UART_MSG_START_ACK);
        handshakeState = HS_ACK_SENT;
        displayDirty = true;
        Serial.println("[SLAVE] 收到 START_REQ (0x02)，已回复 ACK (0x04)，等待 CONFIRM");
    }
    else if (msgType == UART_MSG_START_CONFIRM && len == 0)
    {
        // 三次握手第3步：收到主板 START_CONFIRM (0x05)
        if (handshakeState != HS_DONE)
        {
            flowStarted = true;
            matchStartedAt = millis();
            handshakeState = HS_DONE;
            displayDirty = true;
            Serial.println("[SLAVE] 收到 START_CONFIRM (0x05)，比赛开始！");
        }
        else
        {
            // 已在比赛中，忽略重复帧
            Serial.println("[SLAVE] 收到重复 START_CONFIRM，已忽略");
        }
    }
    else
    {
        Serial.printf("[SLAVE] 收到未知指令: type=0x%02X len=%u\n",
                      msgType, (unsigned)len);
    }
}

void receiveFromMain()
{
    while (Serial2.available())
    {
        uint8_t b = Serial2.read();

        switch (rxState)
        {
        case RX_STATE_WAIT_HEADER0:
            if (b == FRAME_HEADER0)
                rxState = RX_STATE_WAIT_HEADER1;
            break;

        case RX_STATE_WAIT_HEADER1:
            if (b == FRAME_HEADER1)
                rxState = RX_STATE_READ_MSG_TYPE;
            else
                rxState = RX_STATE_WAIT_HEADER0;
            break;

        case RX_STATE_READ_MSG_TYPE:
            rxMsgType = b;
            rxState = RX_STATE_READ_PAYLOAD_LEN;
            break;

        case RX_STATE_READ_PAYLOAD_LEN:
            rxPayloadLen = b;
            rxPayloadIndex = 0;
            rxChecksum = rxMsgType ^ rxPayloadLen;

            if (rxPayloadLen == 0)
            {
                rxState = RX_STATE_READ_CHECKSUM;
            }
            else if (rxPayloadLen > sizeof(rxPayload))
            {
                Serial.printf("[SLAVE] 接收载荷过长: %u\n", (unsigned)rxPayloadLen);
                resetRxState();
            }
            else
            {
                rxState = RX_STATE_READ_PAYLOAD;
            }
            break;

        case RX_STATE_READ_PAYLOAD:
            rxPayload[rxPayloadIndex++] = b;
            rxChecksum ^= b;
            if (rxPayloadIndex >= rxPayloadLen)
                rxState = RX_STATE_READ_CHECKSUM;
            break;

        case RX_STATE_READ_CHECKSUM:
            if (b == rxChecksum)
                rxState = RX_STATE_READ_TAIL;
            else
                resetRxState();
            break;

        case RX_STATE_READ_TAIL:
            if (b == FRAME_TAIL)
                handleRxFrame(rxMsgType, rxPayload, rxPayloadLen);
            resetRxState();
            break;

        default:
            resetRxState();
            break;
        }
    }
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

        // 当前遮挡状态：每轮刷新，供显示
        laserState[i] = triggered;

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

// ==================== 显示刷新：变化触发 + 周期刷新 ====================
void updateDisplayIfNeeded()
{
    unsigned long now = millis();

    // 状态发生变化时立即标脏
    bool changed = (flowStarted != lastFlowStarted) ||
                   (vibrationActive != lastVibrationActive) ||
                   (memcmp(lastLaserState, laserState, sizeof(lastLaserState)) != 0);
    if (changed)
    {
        displayDirty = true;
    }

    // 变化立即刷；无变化时按周期刷，避免偶发漏更新
    if (displayDirty || (now - lastDisplayTime >= DISPLAY_REFRESH_MS))
    {
        unsigned long elapsed = flowStarted ? (now - matchStartedAt) / 1000UL : 0UL;
        displayRender(flowStarted, vibrationActive, laserState, elapsed);

        lastFlowStarted = flowStarted;
        lastVibrationActive = vibrationActive;
        memcpy(lastLaserState, laserState, sizeof(lastLaserState));
        displayDirty = false;
        lastDisplayTime = now;
    }
}

// ==================== 主入口 ====================
void setup()
{
    Serial.begin(115200); // 调试串口
    Serial2.begin(115200, SERIAL_8N1, ReceiveMsgPin, SendMsgPin); // 与主板通信

    // 初始化 NFP1315-157B OLED 显示屏
    displayInit();

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

    delay(100); // 等待主板上电，避免开机时串口数据丢失
    Serial.println("========================================");
    Serial.println("  从板启动：11字节传感器数据上报");
    Serial.println("  payload = isCrashed[7] + isCovered[4]");
    Serial.println("  等待主板 START 指令...");
    Serial.println("========================================");
}

void loop()
{
    // 始终监听主板下发的指令（START 等）
    receiveFromMain();
    // 未收到 START 前：不检测、不上报，只保持显示“等待开始”
    if (!flowStarted)
    {
        updateDisplayIfNeeded();
        delay(1);
        return;
    }

    // 收到 START 后才开始检测传感器并上报
    updateVibration();
    updateLasers();

    uint8_t payload[SENSOR_PAYLOAD_LEN];
    buildPayload(payload);
    sendIfNeeded(payload);

    updateDisplayIfNeeded();

    delay(1); // 轻微让出 CPU；硬件已 RC 消抖，无需长延时
}
