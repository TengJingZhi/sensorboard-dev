将显示屏激光 4 个数字从"被触发过的锁存标志"改为"当前遮挡状态"，UART 上报行为保持不变。

## 改动内容

### 1. src/main.cpp
- 在 `laserAcc` 声明旁（约 main.cpp:55）新增全局变量：
  ```cpp
  // 激光当前遮挡状态：每轮采样刷新，仅供显示
  bool laserState[LASER_COUNT] = {false};
  ```
- `updateLasers()`（main.cpp:255-276）：每轮采样无条件写入 `laserState[i] = triggered;`。`laserAcc` 的"一旦触发保持 1"置位逻辑保留（上报 payload 与串口调试打印仍使用它）。
- `updateDisplayIfNeeded()`（main.cpp:326-351）改用 `laserState`：
  - main.cpp:333 变化检测 memcmp 改为比较 `laserState`；
  - main.cpp:343 `displayRender(...)` 传 `laserState`；
  - main.cpp:347 快照 memcpy 保存 `laserState`；
  - 顺带将 `lastLaserAcc`（main.cpp:73）改名为 `lastLaserState`，语义一致。

### 2. include/display.h
- `displayRender` 参数 `const bool laserAcc[4]` 改名为 `const bool laserState[4]`，注释改为"laserState : 4 路激光当前状态（1=当前被遮挡）"。

### 3. src/display.cpp
- `displayRender` 实现参数同步改名（display.cpp:30-31），注释"激光：累积状态，触发后保持 1"（display.cpp:54）改为"激光：当前状态"。

## 效果
- 屏幕 `Lsr:` 后 4 个数字实时反映各路激光当前是否被遮挡（1=当前遮挡，0=未遮挡），遮挡移除后自动回 0。
- 上报主板的 payload 仍使用 `laserAcc` 累积锁存量，与主板协议行为完全不变。

## 验证
- `pio run` 编译通过。
- 建议烧录后实测：遮挡某路激光对应位变 1，移除后回 0。