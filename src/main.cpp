#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

uint8_t receiver[] = { 0x08, 0xa6, 0xf7, 0x17, 0x6d, 0x84 }; // ESP32_薄
// uint8_t receiver[] = { 0x2c, 0xbc, 0xbb, 0x00, 0x52, 0xd4 }; // ESP32_厚

// #define DEBUG

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

typedef struct {
  float P_roll, I_roll, D_roll,
      P_pitch, I_pitch, D_pitch; // PID参数
} Angle_t;
Angle_t PID;

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

// 串口输出
void SerialPrint() {
  Serial.printf("Roll: %.2f, Gyro Roll: %.2f\n", Gyro_data.angle_roll, Gyro_data.gyro_roll);
  Serial.printf("Pitch: %.2f, Gyro Pitch: %.2f\n", Gyro_data.angle_pitch, Gyro_data.gyro_pitch);
  xSemaphoreGive(mpu6050Mutex);
  vTaskDelay(100);
}

// PID调参
void PID_value_set(void* pvParameters) {
  float step = 0.5;
  while (1) {
    if (Serial.available()) {
      char c = Serial.read();
      switch (c) {
      case 'w': // 增加P
        PID.P_roll += step;
        // PID.P_pitch += step;
        Serial.printf("P_roll: %.2f, I_roll: %.2f, D_roll: %.2f\n", PID.P_roll, PID.I_roll, PID.D_roll);
        break;
      case 's': // 减少P
        PID.P_roll -= step;
        // PID.P_pitch -= step;
        Serial.printf("P_roll: %.2f, I_roll: %.2f, D_roll: %.2f\n", PID.P_roll, PID.I_roll, PID.D_roll);
        break;
      case 'a': // 增加I
        PID.I_roll += step;
        // PID.I_pitch += step;
        Serial.printf("P_roll: %.2f, I_roll: %.2f, D_roll: %.2f\n", PID.P_roll, PID.I_roll, PID.D_roll);
        break;
      case 'd': // 减少I
        PID.I_roll -= step;
        // PID.I_pitch -= step;
        Serial.printf("P_roll: %.2f, I_roll: %.2f, D_roll: %.2f\n", PID.P_roll, PID.I_roll, PID.D_roll);
        break;
      case 'q': // 增加D
        PID.D_roll += step;
        // PID.D_pitch += step;
        Serial.printf("P_roll: %.2f, I_roll: %.2f, D_roll: %.2f\n", PID.P_roll, PID.I_roll, PID.D_roll);
        break;
      case 'e': // 减少D
        PID.D_roll -= step;
        // PID.D_pitch -= step;
        Serial.printf("P_roll: %.2f, I_roll: %.2f, D_roll: %.2f\n", PID.P_roll, PID.I_roll, PID.D_roll);
        break;
      default:
        break;
      }
    }
    if (PID.P_roll < 0) PID.P_roll = 0;
    if (PID.I_roll < 0) PID.I_roll = 0;
    if (PID.D_roll < 0) PID.D_roll = 0;
    if (PID.P_pitch < 0) PID.P_pitch = 0;
    if (PID.I_pitch < 0) PID.I_pitch = 0;
    if (PID.D_pitch < 0) PID.D_pitch = 0;
  }
}

// 读取数据
void getGyroData(void* pvParameters) {
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(5); // 频率 200Hz → 周期为 1/200 = 0.005 秒 = 5 毫秒
  while (1) {
    if (xSemaphoreTake(mpu6050Mutex, portMAX_DELAY)) {
      // 获取陀螺仪数据
      mpu6050.update();
      Gyro_data.angle_roll  = mpu6050.getAngleY();
      Gyro_data.gyro_roll   = mpu6050.getGyroY();
      Gyro_data.angle_pitch = mpu6050.getAngleX();
      Gyro_data.gyro_pitch  = mpu6050.getGyroX();
      xSemaphoreGive(mpu6050Mutex);
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
    } else {
      Serial.println("Failed to take mutex");
    }
  }
}

// 发送数据
void sendData(void* pvParameters) {
  WiFi.mode(WIFI_STA);                  // 设置wifi为STA模式
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  // 注册通信频道
  memcpy(peerInfo.peer_addr, receiver, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                    // 设置通信频道
  esp_now_add_peer(&peerInfo);             // 添加通信对象
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod       = pdMS_TO_TICKS(10); // 频率 100Hz → 周期为 1/100 = 0.01 秒 = 10 毫秒

  while (1) {
    float angle_pitch, angle_roll, gyro_pitch, gyro_roll;
    if (xSemaphoreTake(mpu6050Mutex, 100 / portTICK_PERIOD_MS)) {
      angle_pitch = Gyro_data.angle_pitch;
      angle_roll  = Gyro_data.angle_roll;
      gyro_pitch  = Gyro_data.gyro_pitch;
      gyro_roll   = Gyro_data.gyro_roll;
      xSemaphoreGive(mpu6050Mutex);
    }
    // 发送数据
    esp_now_send(receiver, (uint8_t*)&servoAngle, sizeof(servoAngle));
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup() {
  Serial.begin(115200);

  // 陀螺仪
  Wire.begin(SDA_PIN, SCL_PIN);

  mpu6050Mutex = xSemaphoreCreateMutex();
  xTaskCreate(getGyroData, "getGyroData", 2048, NULL, 1, NULL);
  xTaskCreate(sendData, "sendData", 2048, NULL, 1, NULL);
  xTaskCreate(PID_value_set, "PID_value_set", 2048, NULL, 1, NULL);
}

void loop() {
}
