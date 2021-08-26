#include "./libraries/sd_log.hpp"
#include "./libraries/motor.hpp"
#include "./libraries/gps.hpp"
#include "./libraries/imu.hpp"
#include "./libraries/cansat_io.hpp"
#include "./libraries/log_task.hpp"
#include <cmath>

// 設定
#define ENABLE_GPS true
#define REDUCE_MPU_LOG true
#define SPEED 200
#define YAW_DIFF_THRESHOLD 3.0

// 進む向き
double targetYaw = 0;

SdLog sdLog;
Motor motor;
GPS gps;
IMU imu;
CanSatIO canSatIO;
LogTask logTask(&sdLog, &canSatIO);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("start");

  imu.setup();

  // ゴールの緯度経度
  gps.setGoal(31.5681451, 130.5434225);

  // キャリブレーション結果に値を変更してください
  // imu.mpu.setAccBias(0, 0, 0);
  // imu.mpu.setGyroBias(0, 0, 0);
  // imu.mpu.setMagBias(0, 0, 0);
  // imu.mpu.setMagScale(1, 1, 1);

  // Madgwick filterを使う
  imu.selectMadgwickFilter();

  logTask.setupTask();

  createNewLogFile();

  if (!imu.checkValue(5)) {
    String buffer = "IMU initialization failed.";
    logTask.sendToLoggerTask(buffer, false);
    logTask.restartOnError();
  }

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

int currentSpeed = SPEED;

void loop() {
  int speed_a = 0;
  int speed_b = 0;

  if (canSatIO.isFlightPinInserted()) {
    speed_a = currentSpeed;
    speed_b = currentSpeed;
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
      logTask.sendToLoggerTask(buffer, true);
    }

    if (canSatIO.isButtonJustPressed()) {
      // スイッチが押された
      Serial.println("Create new log file");

      sdLog.closeFile();
      createNewLogFile();

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
      speed_a = currentSpeed - std::abs(diff) * 2;
      speed_b = currentSpeed;
    } else if (YAW_DIFF_THRESHOLD < diff) {
      canSatIO.setLEDOff();
      speed_a = currentSpeed;
      speed_b = currentSpeed - std::abs(diff) * 2;
    } else {
      canSatIO.setLEDOn();
      speed_a = currentSpeed;
      speed_b = currentSpeed;
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
    logTask.sendToLoggerTask(buffer, true);
  }
  if (ENABLE_GPS) {
    String buffer = "";
    gps.readValues(buffer);
    logTask.sendToLoggerTask(buffer, false);
    // ゴールの向きにtargetYawを設定
    targetYaw = gps.courseToGoal;
    if (gps.distanceToGoal < 1.0) {
      currentSpeed = SPEED * 0.25;
    } else if (gps.distanceToGoal < 3.0) {
      currentSpeed = SPEED * 0.5;
    } else {
      currentSpeed = SPEED;
    }
  }
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

  logTask.sendToLoggerTask(message, false);

  imu.mpu.setMagBias(magX, magY, magZ);
}

void createNewLogFile() {
  int current_log_number = sdLog.openNextLogFile(SD);

  Serial.print("Next number: ");
  Serial.println(current_log_number);

  if (ENABLE_GPS) {
    String message = "";
    gps.getHeader(message);
    logTask.sendToLoggerTask(message, false);
  }

  String message = "";
  imu.getHeader(message);

  logTask.sendToLoggerTask(message, false);
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

  logTask.sendToLoggerTask(message, false);

  delay(1000);
}
