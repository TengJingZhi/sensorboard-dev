#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// --- 引脚定义 ---
const int viberatorPins[] = {8,9,10,11,12,13,14};
const int viberatorCount   = sizeof(viberatorPins) / sizeof(viberatorPins[0]);

const int laserPins[] = {4,5,6,7};
const int laserCount   = sizeof(laserPins) / sizeof(laserPins[0]);

const int laserOutputPin = 15;

const int ReceiveMsgPin = 41;
const int SendMsgPin = 42;

// --- 振动检测：全局 1 秒节流 ---
const unsigned long VIBERATION_THROTTLE_MS = 1000;
unsigned long lastViberationTime = 0;

// --- 激光检测：边沿检测状态跟踪 ---
volatile bool lastLaserState[4] = {LOW, LOW, LOW, LOW};  // 初始化为 LOW，表示激光未被遮挡

void setup() {
  Serial.begin(115200);//与电脑通信用的
  Serial2.begin(115200, SERIAL_8N1, ReceiveMsgPin, SendMsgPin);// 与主板通信用的

  // 初始化振动传感器引脚
  for (int i = 0; i < viberatorCount; i++) {
    pinMode(viberatorPins[i], INPUT_PULLDOWN);
  }

  // Initialize the laser pins
  pinMode(laserOutputPin, OUTPUT);
  digitalWrite(laserOutputPin, HIGH);  // 开启激光发射器（若不亮改为 LOW）
  for (int i = 0; i < laserCount; i++) {
    pinMode(laserPins[i], INPUT);
  }
}

void loop() {
  // --- 振动检测（全局 1 秒节流）---
  for (int i = 0; i < viberatorCount; i++) {
    if (digitalRead(viberatorPins[i]) == HIGH) {
      if (millis() - lastViberationTime >= VIBERATION_THROTTLE_MS) {
        Serial.print("Viberation detected! Switch No. ");
        Serial.println(i + 1);
        lastViberationTime = millis();
      }
      break;  // 全局节流，检测到一个即可跳出
    }
  }

  // --- 激光检测（边沿检测去抖）---
  for (int i = 0; i < laserCount; i++) {
    volatile int currentState = analogRead(laserPins[i])>512 ? HIGH : LOW;  // 使用模拟读取来判断激光是否被遮挡
    if (lastLaserState[i] == LOW && currentState == HIGH) {
      Serial.print("Laser Detected! Switch No. ");
      Serial.println(i + 1);
    }
    lastLaserState[i] = currentState;
  }
}
