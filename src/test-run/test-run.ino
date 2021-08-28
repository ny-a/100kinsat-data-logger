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

  if (!logTask.setupTask()) {
    logTask.restartOnError(3);
  }

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

  if (!logTask.checkSdHealth()) {
    Serial.println("SD initialization failed.");
    logTask.restartOnError(6);
  }

  logTask.sendToLoggerTask("State,Wait for GPS fix.\n", false);
  state.enableSdLog = false;
  state.vehicleMode = VehicleMode::Stop;
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
      if (state.vehicleMode != VehicleMode::Stop) {
        logTask.sendToLoggerTask("State,State was changed.\n", false);
        break;
      }
    }
  }
  state.enableSdLog = true;
  if (state.vehicleMode == VehicleMode::Stop) {
    state.vehicleMode = VehicleMode::Mission;
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

      state.vehicleMode = VehicleMode::Mission;

      break;
    }

    // state.targetYaw に向ける
    state.yawDiff = state.clipYawDiff(state.targetYaw - imu.yaw);
    state.checkYawForGpsCompensation();
    if (state.yawDiff < -state.yawDiffThreshold) {
      canSatIO.setLEDOff();
      state.motorLeft = state.currentSpeed - std::abs(state.yawDiff) * 2;
      state.motorRight = state.currentSpeed;
    } else if (state.yawDiffThreshold < state.yawDiff) {
      canSatIO.setLEDOff();
      state.motorLeft = state.currentSpeed;
      state.motorRight = state.currentSpeed - std::abs(state.yawDiff) * 2;
    } else {
      canSatIO.setLEDOn();
      state.motorLeft = state.currentSpeed;
      state.motorRight = state.currentSpeed;
    }

    state.flightPinInserted = canSatIO.isFlightPinInserted();

    state.detectFallDown = imu.roll < -45 || 45 < imu.roll;

    if (
      !state.flightPinInserted ||
      state.detectFallDown ||
      state.vehicleMode == VehicleMode::Stop ||
      state.vehicleMode == VehicleMode::Completed
    ) {
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
  if (state.vehicleMode == VehicleMode::Mission) {
    state.targetYaw = gps.courseToGoal;
    if (gps.distanceToGoal < 0.5) {
      state.currentSpeed = state.defaultSpeed * 0.15;
      // TODO: 終了条件
      // state.vehicleMode = VehicleMode::Completed;
      // logTask.sendToLoggerTask("State,Mission Completed.\n", false);
    } else if (gps.distanceToGoal < 1.0) {
      state.currentSpeed = state.defaultSpeed * 0.33;
    } else if (gps.distanceToGoal < 3.0) {
      state.currentSpeed = state.defaultSpeed * 0.66;
    } else {
      state.currentSpeed = state.defaultSpeed;
    }
    state.doGpsCompensation();
  }
  state.gpsYawDiff = state.clipYawDiff(imu.yaw - gps.course);
  state.goalDistance = gps.distanceToGoal;
  buffer = "";
  state.getLogString(buffer);
  logTask.sendToLoggerTask(buffer, false);

  if (state.vehicleMode == VehicleMode::CalibrateMag) {
    calibrateMag();
  }
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

  message = "";
  state.getHeader(message);
  logTask.sendToLoggerTask(message, false);

  if (!logTask.checkSdHealth()) {
    Serial.println("SD initialization failed.");
    logTask.restartOnError(6);
  }
}

void calibrateMag() {
  motor.stop();
  logTask.sendToLoggerTask("State,CalibrateMag\n", false);
  imu.magCalibrateInitialize();
  unsigned long lastLog = 0;
  while (!canSatIO.isButtonJustPressed()) {
    imu.update();
    if (100 < millis() - lastLog) {
      lastLog = millis();
      String buffer = "";
      imu.getLogString(buffer);
      logTask.sendToLoggerTask(buffer, true);
      buffer = "";
      state.getLogString(buffer);
      logTask.sendToLoggerTask(buffer, true);
    }
  }
  String buffer = "";
  imu.magCalibrateApply(buffer);
  logTask.sendToLoggerTask(buffer, true);
  state.vehicleMode = VehicleMode::Mission;
}
