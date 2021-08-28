#include "./libraries/state.hpp"
#include "./libraries/sd_log.hpp"
#include "./libraries/motor.hpp"
#include "./libraries/gps.hpp"
#include "./libraries/imu.hpp"
#include "./libraries/cansat_io.hpp"
#include "./libraries/log_task.hpp"
#include <cmath>

// 設定
#define ENABLE_GPS_LOG true
#define REDUCE_MPU_LOG true
#define YAW_DIFF_THRESHOLD 3.0

State state;
SdLog sdLog;
Motor motor;
GPS gps(&state);
IMU imu(&state);
CanSatIO canSatIO;
LogTask logTask(&sdLog, &canSatIO, &state);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("start");

  imu.setup();

  // ゴールの緯度経度
  state.setGoal(31.5681451, 130.5434225);

  // キャリブレーション結果に値を変更してください
  // imu.mpu.setAccBias(0, 0, 0);
  // imu.mpu.setGyroBias(0, 0, 0);
  // imu.mpu.setMagBias(0, 0, 0);
  // imu.mpu.setMagScale(1, 1, 1);

  // Madgwick filterを使う
  imu.selectMadgwickFilter();

  logTask.setupTask();

  createNewLogFile();

  logTask.sendToLoggerTask("State,GPS check.\n", false);
  if (!gps.changeUpdateInterval()) {
    logTask.sendToLoggerTask("State,GPS initialization failed.\n", false);
    logTask.restartOnError(4);
  }
  logTask.sendToLoggerTask("State,GPS OK.\n", false);

  if (!imu.checkValue(5)) {
    logTask.sendToLoggerTask("State,IMU initialization failed.\n", false);
    logTask.restartOnError(5);
  }
  logTask.sendToLoggerTask("State,IMU OK.\n", false);

  logTask.sendToLoggerTask("State,Wait for GPS fix.\n", false);
  state.enableSdLog = false;
  for (unsigned long start = millis(), lastLog = 0; millis() - start < 2 * 60 * 1000;) {
    gps.encode();
    if (1000 < millis() - lastLog) {
      lastLog = millis();
      String buffer = "";
      gps.readValues(buffer);
      logTask.sendToLoggerTask(buffer, false);
      if (gps.locationIsValid) {
        logTask.sendToLoggerTask("State,GPS location is valid.\n", false);
        break;
      }
    }
  }
  state.enableSdLog = true;

  if (!canSatIO.isFlightPinInserted()) {
    // フライトピンが抜けている
    canSatIO.setLEDOn();
    // 真っ直ぐ歩いてください
    calibrateNorthByGPS(15);
    canSatIO.setLEDOff();

    delay(1000);
  }

  logTask.sendToLoggerTask("State,setup ended.\n", false);
}

void loop() {
  if (canSatIO.isFlightPinInserted()) {
    state.motorLeft = state.currentSpeed;
    state.motorRight = state.currentSpeed;
  } else {
    state.motorLeft = 0;
    state.motorRight = 0;
  }

  const unsigned long ms = 100;
  unsigned long start = millis();

  while (millis() - start < ms) {
    gps.encode();

    if (imu.update() && !REDUCE_MPU_LOG) {
      String buffer = "";
      imu.getLogString(buffer);
      logTask.sendToLoggerTask(buffer, true);
    }

    if (canSatIO.isButtonJustPressed()) {
      // スイッチが押された
      Serial.println("Create new log file");

      logTask.closeFile();
      createNewLogFile();

      break;
    }

    // state.targetYaw に向ける
    int diff = state.targetYaw - imu.yaw;
    if (diff < -180) {
      diff += 360;
    } else if (180 < diff) {
      diff -= 360;
    }
    if (diff < -YAW_DIFF_THRESHOLD) {
      canSatIO.setLEDOff();
      state.motorLeft = state.currentSpeed - std::abs(diff) * 2;
      state.motorRight = state.currentSpeed;
    } else if (YAW_DIFF_THRESHOLD < diff) {
      canSatIO.setLEDOff();
      state.motorLeft = state.currentSpeed;
      state.motorRight = state.currentSpeed - std::abs(diff) * 2;
    } else {
      canSatIO.setLEDOn();
      state.motorLeft = state.currentSpeed;
      state.motorRight = state.currentSpeed;
    }

    if (
      !canSatIO.isFlightPinInserted() ||
      imu.roll < -45 ||
      45 < imu.roll
    ) {
      // フライトピンが抜かれるか、機体が横転したら止める
      state.motorLeft = 0;
      state.motorRight = 0;
    }
    motor.move(state.motorLeft, state.motorRight);
  }

  if (REDUCE_MPU_LOG) {
    String buffer = "";
    imu.getLogString(buffer);
    logTask.sendToLoggerTask(buffer, true);
  }
  String buffer = "";
  gps.readValues(buffer);
  if (ENABLE_GPS_LOG) {
    logTask.sendToLoggerTask(buffer, false);
  }
  // ゴールの向きにtargetYawを設定
  if (state.targetIsGoal) {
    state.targetYaw = gps.courseToGoal;
    if (gps.distanceToGoal < 1.0) {
      state.currentSpeed = state.defaultSpeed * 0.25;
    } else if (gps.distanceToGoal < 3.0) {
      state.currentSpeed = state.defaultSpeed * 0.5;
    } else {
      state.currentSpeed = state.defaultSpeed;
    }
  }
  buffer = "";
  state.getLogString(buffer);
  logTask.sendToLoggerTask(buffer, false);
}

void fastCalibrate() {
  motor.move(state.currentSpeed, -state.currentSpeed);

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
  int current_log_number = logTask.openNextLogFile(SD);

  Serial.print("Next number: ");
  Serial.println(current_log_number);

  if (ENABLE_GPS_LOG) {
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
  state.northYaw = northYaw;

  String message = "northYaw: ";
  message += String(northYaw, 6);
  message += String("\n");

  logTask.sendToLoggerTask(message, false);

  delay(1000);
}

void calibrateNorthByGPS(int timeoutSeconds) {
  canSatIO.setLEDOn();

  const unsigned long ms = timeoutSeconds * 1000;
  const unsigned long logIntervalMs = 100;
  unsigned long start = millis();
  unsigned long lastLogOutput = start;

  const int coursesMax = 8;
  double courses[coursesMax];
  double yaws[coursesMax];
  int i = 0;

  while (millis() - start < ms) {
    gps.encode();
    imu.update();
    if (canSatIO.isButtonJustPressed()) {
      break;
    }
    if (lastLogOutput < millis() - logIntervalMs) {
      lastLogOutput = millis();
      String buffer = "State: calibrateNorthByGPS\n";
      logTask.sendToLoggerTask(buffer, false);
      buffer = "";
      imu.getLogString(buffer);
      logTask.sendToLoggerTask(buffer, false);
      buffer = "";
      gps.readValues(buffer);
      logTask.sendToLoggerTask(buffer, false);
      courses[i % coursesMax] = gps.courseToGoal;
      yaws[i % coursesMax] = imu.yaw;
      i++;
    }
  }
  canSatIO.setLEDOff();

  double course = 0.0;
  double yaw = 0.0;
  int availableSamples = std::min(i, coursesMax);

  if (availableSamples == 0) {
    String message = "calibrateNorthByGPS cancelled.\n";
    logTask.sendToLoggerTask(message, false);
    return;
  }

  for (int i = 0; i < availableSamples; i++) {
    course += courses[i];
    yaw += yaws[i];
  }

  course /= availableSamples;
  yaw /= availableSamples;

  double northYaw = yaw - course;

  if (northYaw < 0) {
    northYaw += 360.0;
  }

  state.northYaw = northYaw;

  String message = "northYaw: ";
  message += String(northYaw, 6);
  message += String("\n");

  logTask.sendToLoggerTask(message, false);

  delay(1000);
}
