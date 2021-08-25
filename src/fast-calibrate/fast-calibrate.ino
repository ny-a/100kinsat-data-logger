#include "./libraries/sd_log.hpp"
#include "./libraries/motor.hpp"
#include "./libraries/gps.hpp"
#include "./libraries/imu.hpp"
#include "./libraries/cansat_io.hpp"
#include <cmath>

// 設定
#define ENABLE_GPS true
#define REDUCE_MPU_LOG false
#define SPEED 200
#define YAW_DIFF_THRESHOLD 3.0

// 進む向き
double targetYaw = 0;

SdLog sdLog;
Motor motor;
GPS gps;
IMU imu;
CanSatIO canSatIO;

QueueHandle_t queue;

#define QUEUE_BUFFER_SIZE 256

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("start");

  imu.setup();

  // ゴールの緯度経度
  gps.setGoal(35.682716, 139.759955);

  // キャリブレーション結果に値を変更してください
  // imu.mpu.setAccBias(0, 0, 0);
  // imu.mpu.setGyroBias(0, 0, 0);
  // imu.mpu.setMagBias(0, 0, 0);
  // imu.mpu.setMagScale(1, 1, 1);

  // Madgwick filterを使う
  imu.selectMadgwickFilter();

  setupTask();

  createNewLogFile();

  if (!canSatIO.isFlightPinInserted()) {
    // フライトピンが抜けている
    canSatIO.setLEDOn();
    Serial.println("Start Calibrate.");
    // 1周くらい回って簡易的に地磁気センサーをキャリブレーションする
    fastCalibrate();
    canSatIO.setLEDOff();

    delay(1000);

    // 15秒間ボタンが押されるのを待つので北を向けてボタンを押してください
    calibrateNorth(15);
  }
}

void loop() {
  int speed_a = 0;
  int speed_b = 0;

  if (canSatIO.isFlightPinInserted()) {
    speed_a = SPEED;
    speed_b = SPEED;
  } else {
    speed_a = 0;
    speed_b = 0;
  }

  const unsigned long ms = 100;
  unsigned long start = millis();

  while (millis() - start < ms) {
    if (ENABLE_GPS) {
      gps.encode();
    }

    if (imu.update() && !REDUCE_MPU_LOG) {
      String buffer = "";
      imu.getLogString(buffer);
      sendToLoggerTask(buffer, true);
    }

    if (canSatIO.isButtonJustPressed()) {
      // スイッチが押された
      Serial.println("Create new log file");

      sdLog.closeFile();
      createNewLogFile();

      // 反転させる
      if (targetYaw == 0){
        targetYaw = 180;
      } else {
        targetYaw = 0;
      }

      break;
    }

    // targetYaw に向ける
    int diff = targetYaw - imu.yaw;
    if (diff < -180) {
      diff += 360;
    } else if (180 < diff) {
      diff -= 360;
    }
    if (diff < -YAW_DIFF_THRESHOLD) {
      canSatIO.setLEDOff();
      speed_a = SPEED - std::abs(diff) * 2;
      speed_b = SPEED;
    } else if (YAW_DIFF_THRESHOLD < diff) {
      canSatIO.setLEDOff();
      speed_a = SPEED;
      speed_b = SPEED - std::abs(diff) * 2;
    } else {
      canSatIO.setLEDOn();
      speed_a = SPEED;
      speed_b = SPEED;
    }

    if (
      !canSatIO.isFlightPinInserted() ||
      imu.roll < -45 ||
      45 < imu.roll
    ) {
      // フライトピンが抜かれるか、機体が横転したら止める
      speed_a = 0;
      speed_b = 0;
    }
    motor.move(speed_a, speed_b);
  }

  if (REDUCE_MPU_LOG) {
    String buffer = "";
    imu.getLogString(buffer);
    sendToLoggerTask(buffer, true);
  }
  if (ENABLE_GPS) {
    String buffer = "";
    gps.readValues(buffer);
    sendToLoggerTask(buffer, false);
  }
}

void sendToLoggerTask(String &buffer, bool skippable) {
  BaseType_t status;

  if (!skippable || uxQueueMessagesWaiting(queue) == 0) {
    status = xQueueSend(queue, buffer.c_str(), 0);

    // if (status != pdPASS) {
    //   Serial.println("rtos queue send error");
    // }
  }
}

void setupTask() {
  queue = xQueueCreate(64, QUEUE_BUFFER_SIZE);

  if(queue != NULL) {
    xTaskCreatePinnedToCore(loggerTask, "loggerTask", 4096, NULL, 1, NULL, 1);
  } else {
    Serial.println("rtos queue create error, stopped");
    restartOnError();
  }
}

void loggerTask(void *pvParameters) {
  BaseType_t status;
  int writeFailedCount = 0;
  char buffer[QUEUE_BUFFER_SIZE];
  const TickType_t tick = 500U; // [ms]

  while (true) {
    status = xQueueReceive(queue, buffer, tick);
    if(status == pdPASS) {
      Serial.print(buffer);
      bool writeStatus = sdLog.writeLog(buffer);
      if (writeStatus) {
        writeFailedCount = 0;
      } else {
        writeFailedCount++;
      }
    } else {
      if (uxQueueMessagesWaiting(queue) != 0) {
        Serial.println("rtos queue receive error, stopped");
        restartOnError();
      }
    }
    if (10 < writeFailedCount) {
      Serial.println("sd write failed, stopped");
      restartOnError();
    }
  }
}

void restartOnError() {
  for (int i = 0; i < 10; i++) {
    canSatIO.setLEDOn();
    delay(500);
    canSatIO.setLEDOff();
    delay(500);
  }
  Serial.println("Restarting...");
  delay(100);
  ESP.restart();
}

void fastCalibrate() {
  motor.move(SPEED, -SPEED);

  double some_small_value = -1000.0;
  double some_big_value = 1000.0;

  double mag_x_max = some_small_value;
  double mag_x_min = some_big_value;
  double mag_y_max = some_small_value;
  double mag_y_min = some_big_value;
  double mag_z_max = some_small_value;
  double mag_z_min = some_big_value;

  const unsigned long ms = 10000;
  unsigned long start = millis();

  while (millis() - start < ms) {
    if (imu.update()) {
      mag_x_max = std::max(imu.magX, mag_x_max);
      mag_x_min = std::min(imu.magX, mag_x_min);
      mag_y_max = std::max(imu.magY, mag_y_max);
      mag_y_min = std::min(imu.magY, mag_y_min);
      mag_z_max = std::max(imu.magZ, mag_z_max);
      mag_z_min = std::min(imu.magZ, mag_z_min);
    }

    if (canSatIO.isFlightPinInserted()) {
      // フライトピンが刺されたら止める
      break;
    }
  }

  motor.stop();

  String message = "";
  message += String("Calibrate,setMagBias(");
  double magX = imu.mpu.getMagBiasX() + ((mag_x_max + mag_x_min) / 2);
  message += String(magX, 6);
  message += String(", ");
  double magY = imu.mpu.getMagBiasY() + ((mag_y_max + mag_y_min) / 2);
  message += String(magY, 6);
  message += String(", ");
  double magZ = imu.mpu.getMagBiasZ() + ((mag_z_max + mag_z_min) / 2);
  message += String(magZ, 6);
  message += String(");\n");

  sendToLoggerTask(message, false);

  imu.mpu.setMagBias(magX, magY, magZ);
}

void createNewLogFile() {
  int current_log_number = sdLog.openNextLogFile(SD);

  Serial.print("Next number: ");
  Serial.println(current_log_number);

  String message = "";

  if (ENABLE_GPS) {
    gps.getHeader(message);
  }

  imu.getHeader(message);

  sendToLoggerTask(message, false);
}

void calibrateNorth(int timeoutSeconds) {
  double northYaw = 0.0;
  canSatIO.setLEDOn();

  const unsigned long ms = timeoutSeconds * 1000;
  const unsigned long logIntervalMs = 100;
  unsigned long start = millis();
  unsigned long lastLogOutput = start;

  int seconds = 0;
  while (millis() - start < ms) {
    if (imu.update()) {
      northYaw = imu.yaw;
    }
    if (canSatIO.isButtonJustPressed()) {
      break;
    }
    if (lastLogOutput < millis() - logIntervalMs) {
      lastLogOutput = millis();
      Serial.println(northYaw);
    }
  }
  canSatIO.setLEDOff();
  imu.setNorthYaw(northYaw);

  String message = "northYaw: ";
  message += String(northYaw, 6);
  message += String("\n");

  sendToLoggerTask(message, false);

  delay(1000);
}
