#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

uint8_t receiver[] = { 0xf0, 0xf5, 0xbd, 0xb5, 0x60, 0x54 }; // ESP32C3_舵机排针
// uint8_t receiver[] = { 0x08, 0xa6, 0xf7, 0x17, 0x6d, 0x84 }; // ESP32_薄
// uint8_t receiver[] = { 0x2c, 0xbc, 0xbb, 0x00, 0x52, 0xd4 }; // ESP32_厚

// #define DEBUG
#define SDA_PIN 18
#define SCL_PIN 23
#define ALPHA 0.1

typedef struct {
  float roll, pitch;
} RC_t;
RC_t GyroServoAngle;

MPU6050           mpu6050(Wire);
SemaphoreHandle_t mpu6050Mutex = NULL;

// 数据发出去之后的回调函数
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
  } else {
  }
}

// 收到消息后的回调
void OnDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // memcpy(&GyroServoAngle, data, sizeof(GyroServoAngle)); // 将收到的消息进行存储
}

// 串口输出
void SerialPrint(float roll, float pitch) {
  Serial.printf("Roll: %.2f\n", roll);
  // Serial.printf("Pitch: %.2f", pitch);
  // Serial.printf("roll gyro: %.2f\n", mpu6050.getGyroY());
}

// 发送数据
void sendGyroData(void* pvParameters) {
  WiFi.mode(WIFI_STA);                  // 设置wifi为STA模式
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  // 注册通信频道
  memcpy(peerInfo.peer_addr, receiver, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                    // 设置通信频道
  esp_now_add_peer(&peerInfo);             // 添加通信对象

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(10); // 频率 100Hz → 周期为 1/100 = 0.01 秒 = 10 毫秒

  while (1) {
    float pitch_raw, roll_raw, pitch_gyro, roll_gyro;
    mpu6050.update();
    pitch_raw = map(mpu6050.getAngleX(), -180, 180, 0, 120);
    roll_raw  = map(mpu6050.getAngleY(), -180, 180, 0, 120);
    if (roll_raw > 62) {
      GyroServoAngle.roll = roll_raw + (roll_raw - 60) * 2;
    } else if (roll_raw < 58) {
      GyroServoAngle.roll = roll_raw - (60 - roll_raw) * 2;
    } else {
      GyroServoAngle.roll = roll_raw;
    }

    if (pitch_raw > 62) {
      GyroServoAngle.pitch = pitch_raw + (pitch_raw - 60) * 2;
    } else if (pitch_raw < 58) {
      GyroServoAngle.pitch = pitch_raw - (60 - pitch_raw) * 2;
    } else {
      GyroServoAngle.pitch = pitch_raw;
    }

#ifdef DEBUG
    Serial.printf("roll: %.2f", GyroServoAngle.roll);
    Serial.printf("        pitch: %.2f\n", GyroServoAngle.pitch);
#endif

    // 发送数据
    esp_now_send(receiver, (uint8_t*)&GyroServoAngle, sizeof(GyroServoAngle));
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  // mpu6050Mutex = xSemaphoreCreateMutex();
  xTaskCreate(sendGyroData, "sendGyroData", 2048, NULL, 1, NULL);
  // xTaskCreate(PID_value_set, "PID_value_set", 2048, NULL, 1, NULL);
}

void loop() {
}
