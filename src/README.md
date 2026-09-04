# Sensorboard-Dev 源码说明

> 本文档总结从板（ESP32-S3）传感器上报系统的完整技术细节，包括引脚分配、状态变量、核心函数、评分规则宏定义、整体逻辑、技术栈与修改历程。

---

## 目录

- [1. 技术栈与开发环境](#1-技术栈与开发环境)
- [2. 引脚分配](#2-引脚分配)
- [3. 系统状态变量](#3-系统状态变量)
- [4. 评分规则与协议宏定义](#4-评分规则与协议宏定义)
- [5. 核心函数](#5-核心函数)
- [6. 整体逻辑实现](#6-整体逻辑实现)
- [7. 修改历程](#7-修改历程)

---

## 1. 技术栈与开发环境

| 项目           | 说明                                                                               |
| -------------- | ---------------------------------------------------------------------------------- |
| **MCU**        | ESP32-S3-DevKitC-1                                                                 |
| **框架**       | Arduino (PlatformIO)                                                               |
| **平台**       | Espressif32                                                                        |
| **Flash**      | 16MB（DIO 模式）                                                                   |
| **PSRAM**      | OPI 模式，已启用                                                                   |
| **USB 模式**   | CDC 串口（`ARDUINO_USB_MODE=1`, `ARDUINO_USB_CDC_ON_BOOT=1`）                      |
| **串口监视器** | 115200 baud                                                                        |
| **依赖库**     | `Adafruit NeoPixel@^1.12.2`（板载 RGB LED）、`olikraus/U8g2@^2.35.19`（OLED 显示） |
| **显示屏**     | NFP1315-157B（1.57 寸 OLED，SSD1315 驱动，128×64，I2C）                            |
| **开发工具**   | VS Code + PlatformIO                                                               |

### 编译标志（`platformio.ini`）

```ini
build_flags =
  -DBOARD_HAS_PSRAM
  -DARDUINO_USB_MODE=1
  -DARDUINO_USB_CDC_ON_BOOT=1
```

---

## 2. 引脚分配

### 2.1 传感器与通信引脚

| 功能              | GPIO | 方向           | 说明                          |
| ----------------- | ---- | -------------- | ----------------------------- |
| 振动开关 1        | 8    | INPUT_PULLDOWN | `viberatorPins[0]`            |
| 振动开关 2        | 9    | INPUT_PULLDOWN | `viberatorPins[1]`            |
| 振动开关 3        | 10   | INPUT_PULLDOWN | `viberatorPins[2]`            |
| 振动开关 4        | 11   | INPUT_PULLDOWN | `viberatorPins[3]`            |
| 振动开关 5        | 12   | INPUT_PULLDOWN | `viberatorPins[4]`            |
| 振动开关 6        | 13   | INPUT_PULLDOWN | `viberatorPins[5]`            |
| 振动开关 7        | 14   | INPUT_PULLDOWN | `viberatorPins[6]`            |
| 激光接收 1        | 4    | INPUT (ADC)    | `laserPins[0]`                |
| 激光接收 2        | 5    | INPUT (ADC)    | `laserPins[1]`                |
| 激光接收 3        | 6    | INPUT (ADC)    | `laserPins[2]`                |
| 激光接收 4        | 7    | INPUT (ADC)    | `laserPins[3]`                |
| 激光发射控制      | 15   | OUTPUT         | `laserOutputPin`，HIGH = 开启 |
| UART RX（接主板） | 41   | —              | `ReceiveMsgPin`（Serial2 RX） |
| UART TX（接主板） | 42   | —              | `SendMsgPin`（Serial2 TX）    |

### 2.2 OLED 显示引脚

| 显示屏 | ESP32-S3 | 宏定义            |
| ------ | -------- | ----------------- |
| SDA    | GPIO 16  | `DISPLAY_SDA_PIN` |
| SCL    | GPIO 17  | `DISPLAY_SCL_PIN` |
| VCC    | 3.3V     | —                 |
| GND    | GND      | —                 |
| RESET  | 不连接   | `U8X8_PIN_NONE`   |

> **引脚占用说明**：GPIO 4~15 已用于传感器与激光，41/42 用于 UART 通信，因此 I2C 显示选在 16/17。

### 2.3 引脚数组定义（`main.cpp`）

```cpp
const int viberatorPins[] = {8, 9, 10, 11, 12, 13, 14};
#define VIBERATION_COUNT 7

const int laserPins[] = {4, 5, 6, 7};
#define LASER_COUNT 4

const int laserOutputPin = 15;
const int ReceiveMsgPin = 41;
const int SendMsgPin = 42;
```

---

## 3. 系统状态变量

### 3.1 激光状态

| 变量            | 类型   | 说明                                                    |
| --------------- | ------ | ------------------------------------------------------- |
| `laserAcc[4]`   | `bool` | 激光累积触发历史，一旦触发永久保持 `true`，用于组帧上报 |
| `laserState[4]` | `bool` | 激光当前遮挡状态，每轮采样刷新，仅供显示                |

### 3.2 振动状态

| 变量                  | 类型            | 说明                                               |
| --------------------- | --------------- | -------------------------------------------------- |
| `vibrationActive`     | `int`           | 当前锁存的振动开关编号（0~6），`-1` 表示无振动事件 |
| `vibrationBlockUntil` | `unsigned long` | 振动屏蔽截止时间（`millis()` 基准）                |

### 3.3 通信与发送状态

| 变量                  | 类型             | 说明                                                  |
| --------------------- | ---------------- | ----------------------------------------------------- |
| `lastSentPayload[11]` | `uint8_t`        | 上次实际发送的 payload，用于"变化触发"比较            |
| `lastSendTime`        | `unsigned long`  | 上次发送时间戳                                        |
| `flowStarted`         | `bool`           | 流程开始标志，收到主板 `START_CONFIRM` 后置 `true`    |
| `matchStartedAt`      | `unsigned long`  | 比赛开始时间戳，用于计算已运行秒数                    |
| `handshakeState`      | `HandshakeState` | 三次握手状态机：`HS_IDLE` → `HS_ACK_SENT` → `HS_DONE` |

### 3.4 接收状态机变量

| 变量             | 类型      | 说明                   |
| ---------------- | --------- | ---------------------- |
| `rxState`        | `RxState` | 接收解析状态机当前状态 |
| `rxMsgType`      | `uint8_t` | 当前帧消息类型         |
| `rxPayloadLen`   | `uint8_t` | 当前帧 payload 长度    |
| `rxPayload[16]`  | `uint8_t` | payload 缓冲区         |
| `rxPayloadIndex` | `size_t`  | payload 已读字节数     |
| `rxChecksum`     | `uint8_t` | 累积校验和             |

### 3.5 显示状态

| 变量                  | 类型            | 说明                                   |
| --------------------- | --------------- | -------------------------------------- |
| `lastFlowStarted`     | `bool`          | 上次显示的 `flowStarted`，用于变化检测 |
| `lastVibrationActive` | `int`           | 上次显示的振动编号                     |
| `lastLaserState[4]`   | `bool`          | 上次显示的激光状态                     |
| `displayDirty`        | `bool`          | 显示脏标志，`true` 表示需要刷新        |
| `lastDisplayTime`     | `unsigned long` | 上次显示刷新时间戳                     |

---

## 4. 评分规则与协议宏定义

### 4.1 可调参数宏（`main.cpp`）

| 宏名                   | 默认值   | 说明                                                             |
| ---------------------- | -------- | ---------------------------------------------------------------- |
| `VIBRATION_BLOCK_MS`   | `2000UL` | 振动屏蔽时间：该时间内锁存最先触发的振动位并持续上报为 1         |
| `HEARTBEAT_MS`         | `200UL`  | 心跳发送周期：数据无变化时也按此周期发送一次                     |
| `MIN_SEND_INTERVAL_MS` | `10UL`   | 最小发送间隔：防止快速抖动导致总线拥塞                           |
| `LASER_THRESHOLD`      | `2048`   | 激光遮挡判定阈值（ADC 0~4095 的中值）                            |
| `LASER_ACTIVE_HIGH`    | `1`      | 激光被遮挡时输出高电平（`1`=高电平表示遮挡，`0`=低电平表示遮挡） |
| `DISPLAY_REFRESH_MS`   | `500UL`  | 显示周期兜底刷新间隔                                             |

> 所有宏均可通过 `platformio.ini` 的 `build_flags` 覆盖，无需修改源码。

### 4.2 UART 协议宏

| 宏名                     | 值     | 说明                                   |
| ------------------------ | ------ | -------------------------------------- |
| `FRAME_HEADER0`          | `0xAA` | 帧头第 1 字节                          |
| `FRAME_HEADER1`          | `0x55` | 帧头第 2 字节                          |
| `FRAME_TAIL`             | `0x0D` | 帧尾                                   |
| `UART_MSG_SENSOR_DATA`   | `0x01` | 传感器数据帧（从板→主板）              |
| `UART_MSG_START`         | `0x02` | 开始请求（主板→从板，三次握手第 1 步） |
| `UART_MSG_START_ACK`     | `0x04` | 确认就绪（从板→主板，三次握手第 2 步） |
| `UART_MSG_START_CONFIRM` | `0x05` | 最终确认（主板→从板，三次握手第 3 步） |
| `SENSOR_PAYLOAD_LEN`     | `11`   | payload 长度 = 7（振动）+ 4（激光）    |

### 4.3 帧格式

#### 传感器数据帧（11 字节 payload）

```
| 0xAA | 0x55 | 0x01 | 0x0B | payload[0..10] | checksum | 0x0D |
```

**payload 结构：**

```
payload[0..6]  = isCrashed[0..6]   // 7 个振动开关，屏蔽期内持续为 1
payload[7..10] = isCovered[0..3]   // 4 个激光开关，触发后累积保持为 1
```

**校验和：** `checksum = msgType ^ payloadLen ^ payload[0] ^ ... ^ payload[10]`

#### 控制帧（0 字节 payload，用于三次握手）

```
| 0xAA | 0x55 | msgType | 0x00 | checksum | 0x0D |
```

其中 `checksum = msgType ^ 0x00 = msgType`。

### 4.4 三次握手流程

```
主板(esp-cam)                       从板(sensorboard-dev)
    │                                   │
    │── 0x02 START_REQ ───────────────→│  第1步：主板请求开始
    │                                   │
    │←──────────── 0x04 START_ACK ────│  第2步：从板确认就绪
    │                                   │
    │── 0x05 START_CONFIRM ───────────→│  第3步：主板最终确认
    │                                   │
    │           flowStarted = true      │
    │           开始处理传感器数据       │
```

---

## 5. 核心函数

### 5.1 生命周期函数

| 函数      | 文件       | 说明                                                    |
| --------- | ---------- | ------------------------------------------------------- |
| `setup()` | `main.cpp` | 初始化串口、OLED、传感器引脚；上电后等待主板 START 指令 |
| `loop()`  | `main.cpp` | 主循环：接收指令 → 检测传感器 → 组帧 → 发送 → 刷新显示  |

### 5.2 通信函数

| 函数                                   | 文件       | 说明                                                             |
| -------------------------------------- | ---------- | ---------------------------------------------------------------- |
| `sendSensorFrame(payload)`             | `main.cpp` | 发送 11 字节传感器数据帧，同时打印调试日志                       |
| `sendControlFrame(msgType)`            | `main.cpp` | 发送 0-payload 控制帧（用于握手 ACK）                            |
| `receiveFromMain()`                    | `main.cpp` | 状态机方式接收并解析主板下发的指令帧                             |
| `handleRxFrame(msgType, payload, len)` | `main.cpp` | 处理已解析的接收帧：START_REQ → 回 ACK；START_CONFIRM → 开始比赛 |
| `resetRxState()`                       | `main.cpp` | 重置接收状态机到等待帧头状态                                     |

### 5.3 传感器处理函数

| 函数                    | 文件       | 说明                                          |
| ----------------------- | ---------- | --------------------------------------------- |
| `updateVibration()`     | `main.cpp` | 读取 7 路振动开关；屏蔽期内锁存最先触发的一路 |
| `updateLasers()`        | `main.cpp` | 读取 4 路激光 ADC；更新当前状态与累积历史     |
| `buildPayload(payload)` | `main.cpp` | 组帧：振动位（仅锁存位为 1）+ 激光累积位      |

### 5.4 调度函数

| 函数                      | 文件       | 说明                                                      |
| ------------------------- | ---------- | --------------------------------------------------------- |
| `sendIfNeeded(payload)`   | `main.cpp` | 发送调度：变化立即发 + 无变化按心跳周期发，受最小间隔保护 |
| `updateDisplayIfNeeded()` | `main.cpp` | 显示调度：变化立即刷 + 无变化按 500ms 周期兜底刷新        |
| `resetMatchState()`       | `main.cpp` | 重置全部比赛状态（用于比赛重开场景）                      |

### 5.5 显示函数

| 函数                                                                       | 文件          | 说明                                 |
| -------------------------------------------------------------------------- | ------------- | ------------------------------------ |
| `displayInit()`                                                            | `display.cpp` | 初始化 U8g2 并显示启动画面           |
| `displayRender(matchStarted, vibrationActive, laserState, elapsedSeconds)` | `display.cpp` | 渲染比赛信息：状态行、振动位、激光位 |

### 5.6 函数调用关系

```
setup()
  ├── Serial.begin(115200)          // 调试串口
  ├── Serial2.begin(...)            // 主板通信串口
  ├── displayInit()                 // OLED 初始化
  └── pinMode() × N                 // 传感器引脚初始化

loop()
  ├── receiveFromMain()              // 始终监听主板指令
  │     └── handleRxFrame()
  │           ├── sendControlFrame(START_ACK)
  │           └── resetMatchState()  // 重开场景
  │
  ├── [flowStarted == false] → updateDisplayIfNeeded() → return
  │
  ├── updateVibration()              // 振动采样与锁存
  ├── updateLasers()                 // 激光采样与累积
  ├── buildPayload()                 // 组帧
  ├── sendIfNeeded()                 // 发送调度
  │     └── sendSensorFrame()
  └── updateDisplayIfNeeded()        // 显示调度
        └── displayRender()
```

---

## 6. 整体逻辑实现

### 6.1 系统状态机

```
┌─────────────┐     收到 0x02 START_REQ      ┌──────────────┐
│   HS_IDLE    │ ──────────────────────────→ │ HS_ACK_SENT  │
│  等待开始    │   回复 0x04 START_ACK        │ 等待最终确认  │
└─────────────┘                               └──────┬───────┘
      ↑                                              │
      │ 收到 0x02 时                                  │ 收到 0x05 START_CONFIRM
      │ (已在比赛中则先重置)                            ▼
      │                                        ┌──────────────┐
      └────────────────────────────────────────│   HS_DONE     │
                                               │  比赛进行中   │
                                               └──────────────┘
```

### 6.2 主循环逻辑（`loop()`）

1. **始终监听**：`receiveFromMain()` 解析主板下发的指令帧。
2. **未开始时**：`flowStarted == false`，只刷新显示（显示 `WAIT`），不检测传感器、不上报。
3. **开始后**：
   - `updateVibration()`：读取 7 路振动开关
     - 屏蔽期结束 → 清除锁存（`vibrationActive = -1`）
     - 无锁存时 → 锁存最先触发的一路，设置屏蔽期 `VIBRATION_BLOCK_MS`
   - `updateLasers()`：读取 4 路激光 ADC
     - 与 `LASER_THRESHOLD` 比较，判定是否被遮挡
     - 更新 `laserState`（当前状态，供显示）
     - 更新 `laserAcc`（累积历史，供上报，一旦触发永久保持）
   - `buildPayload()`：组帧
     - 振动位：仅当前锁存的那一路为 1，其余为 0
     - 激光位：使用累积触发历史
   - `sendIfNeeded()`：发送调度
     - 受 `MIN_SEND_INTERVAL_MS` 保护
     - 数据有变化 → 立即发送
     - 无变化 → 按 `HEARTBEAT_MS` 周期发送心跳
   - `updateDisplayIfNeeded()`：显示调度
     - 状态变化 → 立即刷新
     - 无变化 → 按 `DISPLAY_REFRESH_MS` 周期兜底刷新

### 6.3 振动屏蔽机制

```
时间轴：
  ──────────────────────────────────────────────────→
  │          │<── VIBRATION_BLOCK_MS ──>│          │
  │          │                           │          │
  │  振动触发 │  锁存该路，持续上报为 1    │ 屏蔽结束  │
  │          │  其他路为 0               │ 清除锁存  │
  │          │                           │          │
```

- 屏蔽期内只上报锁存的那一路为 1，防止多次触发导致主板重复处理。
- 屏蔽期结束后恢复检测，等待下一次振动事件。

### 6.4 激光累积机制

```
激光触发前：laserAcc[i] = 0
激光触发后：laserAcc[i] = 1 （永久保持，直到 resetMatchState()）
```

- 激光一旦被遮挡，对应位永久保持为 1。
- 上报使用累积值，确保主板不会漏掉历史触发。
- 仅 `resetMatchState()` 可清除累积值（比赛重开时）。

### 6.5 发送调度策略

```
发送条件：
  1. 距上次发送 >= MIN_SEND_INTERVAL_MS (10ms)
  2. 且满足以下之一：
     a. payload 与上次不同（变化触发）
     b. 距上次发送 >= HEARTBEAT_MS (200ms)（周期心跳）
```

### 6.6 显示布局

```
┌────────────────────────────┐
│ Status: RUN 12s           │  ← 比赛状态 + 运行秒数（或 Status: WAIT）
│ Vib: 0001000              │  ← 7 位振动状态（1=当前锁存）
│ Lsr: 1010                 │  ← 4 位激光状态（1=当前被遮挡）
│ Auto refresh              │  ← 提示行
└────────────────────────────┘
```

### 6.7 接收状态机

```
WAIT_HEADER0 → WAIT_HEADER1 → READ_MSG_TYPE → READ_PAYLOAD_LEN
                                                      │
                                    ┌─────────────────┼─────────────────┐
                                    │                 │                 │
                              len == 0          0 < len <= 16       len > 16
                                    │                 │                 │
                                    │                 │                 → reset
                                    ▼                 ▼
                              READ_CHECKSUM ← READ_PAYLOAD (逐字节累积校验)
                                    │
                              checksum 匹配?
                                    │
                                    ▼
                              READ_TAIL → handleRxFrame() → reset
```

---

## 7. 修改历程

### v1.0 — 基础通信验证

- 创建 `communication.cpp` 作为 UART 通信验证脚本
- 实现基础的 `Serial2` 收发测试（文本协议 `MSG:%d|Hello`）
- 确认 ESP32-S3 与主板（esp-cam）之间的 UART 通信链路可用

### v1.1 — 传感器数据上报

- 新增 `src/main.cpp`，实现 7 路振动 + 4 路激光的传感器数据采集
- 定义 11 字节 payload 格式：`isCrashed[7] + isCovered[4]`
- 实现二进制帧协议（帧头 `0xAA 0x55` + 消息类型 + 长度 + payload + 校验和 + 帧尾 `0x0D`）
- 振动开关使用 `INPUT_PULLDOWN`，硬件 RC 消抖

### v1.2 — 振动屏蔽机制

- 新增 `VIBRATION_BLOCK_MS`（默认 2000ms）振动屏蔽时间
- 屏蔽期内锁存最先触发的振动位并持续上报为 1，防止主板重复处理同一振动事件
- 屏蔽期结束后自动清除锁存，恢复检测

### v1.3 — 激光累积触发

- 激光从"即时状态"改为"累积触发历史"
- 一旦触发永久保持为 1，确保主板不漏掉历史触发
- 同时维护 `laserState`（当前状态）供显示使用

### v1.4 — 变化触发 + 周期心跳发送

- 新增 `HEARTBEAT_MS`（200ms）心跳周期和 `MIN_SEND_INTERVAL_MS`（10ms）最小发送间隔
- 数据有变化时立即发送，无变化时按心跳周期发送
- 防止快速抖动导致总线拥塞，同时保证主板定期收到数据

### v1.5 — 三次握手流程

- 新增三次握手状态机：`HS_IDLE` → `HS_ACK_SENT` → `HS_DONE`
- 从板上电后只监听、不发送，收到主板 `START_REQ`（0x02）后才回复 `ACK`（0x04）
- 收到 `START_CONFIRM`（0x05）后才开始正式上报传感器数据
- 支持比赛重开：收到重复 `START_REQ` 时先 `resetMatchState()` 再回 ACK
- 新增 `resetMatchState()` 函数，重置全部比赛状态

### v1.6 — NFP1315-157B OLED 显示

- 新增 `include/display.h` 和 `src/display.cpp`
- 接入 NFP1315-157B（SSD1315 驱动，128×64，I2C）显示屏
- 使用 U8g2 库驱动，I2C 引脚选在 GPIO 16/17（避开已占用的 4~15、41、42）
- 显示内容：比赛状态（WAIT/RUN + 秒数）、7 路振动状态、4 路激光状态
- 显示刷新策略：变化触发 + 500ms 周期兜底刷新

### v1.7 — 接收状态机完善

- 实现完整的帧解析状态机（7 个状态：等待帧头 → 消息类型 → 长度 → payload → 校验 → 帧尾）
- 支持变长 payload，payload 过长时自动丢弃并重置
- 校验和错误时自动重置状态机

### 当前版本状态

- ✅ 11 字节传感器数据上报（7 振动 + 4 激光）
- ✅ 三次握手流程（START_REQ → START_ACK → START_CONFIRM）
- ✅ 振动屏蔽机制（2000ms）
- ✅ 激光累积触发
- ✅ 变化触发 + 周期心跳发送
- ✅ NFP1315-157B OLED 实时显示
- ✅ 接收状态机帧解析
- ✅ 比赛重开支持

### 后续待办（主板侧）

详见 `docs/主板待改事项.md`：

- 主板接收方式改为中断接收 + 快照
- 主板振动事件去重（避免屏蔽期内重复处理）
- 主板激光防循环拍照
- 主板 `regionCorners` 和 `CrashPos` 标定
