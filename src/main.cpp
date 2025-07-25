#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

uint8_t Receiver[] = { 0x08, 0xa6, 0xf7, 0x17, 0x6d, 0x84 }; // ESP32_薄
// uint8_t Receiver[] = { 0x2c, 0xbc, 0xbb, 0x00, 0x52, 0xd4 }; // ESP32_厚

#define SDA_PIN 18
#define SCL_PIN 23

typedef struct {
  int roll, pitch;
} RC_t;
RC_t servoAngle;

typedef struct {
  float
      angle_roll,  // Y轴(滚转)角度
      gyro_roll,   // Y轴(滚转)角速度
      angle_pitch, // X轴(俯仰)角度
      gyro_pitch;  // X轴(俯仰)角速度
} Gyro_t;
Gyro_t Gyro_data;

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
  // memcpy(&servoAngle, data, sizeof(servoAngle)); // 将收到的消息进行存储
}

void getGyroData(void* pvParameters) {
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(5); // 频率 200Hz → 周期为 1/200 = 0.005 秒 = 5 毫秒
  while (1) {
    mpu6050.update();
    Gyro_data.angle_roll  = mpu6050.getAngleY();
    Gyro_data.gyro_roll   = mpu6050.getGyroY();
    Gyro_data.angle_pitch = mpu6050.getAngleX();
    Gyro_data.gyro_pitch  = mpu6050.getGyroX();
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void sendData(void* pvParameters) {
  WiFi.mode(WIFI_STA);                  // 设置wifi为STA模式
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  // 注册通信频道
  memcpy(peerInfo.peer_addr, Receiver, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                    // 设置通信频道
  esp_now_add_peer(&peerInfo);             // 添加通信对象

#ifdef DEBUG
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
  } else {
    Serial.println("ESP-NOW initialized successfully");
  }

#endif
}

void setup() {
  Serial.begin(115200);

  // 陀螺仪
  Wire.begin(SDA_PIN, SCL_PIN);

  mpu6050Mutex = xSemaphoreCreateMutex();
  xTaskCreate(getGyroData, "getGyroData", 2048, NULL, 1, NULL);
  xTaskCreate(sendData, "sendData", 2048, NULL, 1, NULL); // 创建获取陀螺仪数据的任务

#ifdef DEBUG
  if (mpu6050Mutex == NULL) {
    Serial.println("Failed to create mutex");
  } else {
    Serial.println("Mutex created successfully");
  }
  if (getGyroData == NULL) {
    Serial.println("Failed to create getGyroData task");
  } else {
    Serial.println("getGyroData task created successfully");
  }
  if (sendData == NULL) {
    Serial.println("Failed to create sendData task");
  } else {
    Serial.println("sendData task created successfully");
  }
#endif
}

void loop() {
}
