#include "cansat_sd.hpp"
#include <TinyGPS++.h>
#include "MPU9250.h"
#include "motor.hpp"
#include <cmath>

static const bool ENABLE_GPS = false;

static const int TURN_SPEED = 32;

// ゴールの緯度経度
static const double GOAL_LAT = 35.682716, GOAL_LON = 139.759955;

static const char* LOG_DIR = "/log";
static const char* PREVIOUS_NUMBER_FILE = "/prev_log_number.txt";

CanSatSd *sd;

static const int GPS_RX_Pin = 2;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial ss(GPS_RX_Pin);

MPU9250 mpu;

Motor motor = Motor();

static const uint8_t flight_pin = 34;
static const uint8_t button = 35;

int current_log_number = 0;

bool is_button_pressing = false;
String log_filename = "";

bool rotate_cw = true;

void setup() {
  sd = new CanSatSd();
  Serial.println("start");

  if (ENABLE_GPS) {
    ss.begin(GPSBaud);
  }

  Wire.begin();
  delay(2000);

  mpu.setup(0x68);

  // キャリブレーション結果に値を変更してください
  // mpu.setAccBias(0, 0, 0);
  // mpu.setGyroBias(0, 0, 0);
  // mpu.setMagBias(0, 0, 0);
  // mpu.setMagScale(1, 1, 1);

  // Madgwick filterを使う
  mpu.selectFilter(QuatFilterSel::MADGWICK);

  pinMode(flight_pin, INPUT);
  pinMode(button, INPUT); // スイッチを入力モードに設定

  if (!sd->existDir(SD, LOG_DIR)) {
    Serial.println("Creating LOG_DIR...");
    sd->createDir(SD, LOG_DIR);
  }
  sd->listDir(SD, "/", 1);

  int previous_number = sd->readFileInt(SD, PREVIOUS_NUMBER_FILE);
  current_log_number = previous_number;
  Serial.print("Previous log number: ");
  Serial.println(previous_number);

  createNewLogFile();
}


void loop()
{
  bool renew_log_file = false;
  String buffer = "";

  if (digitalRead(flight_pin)) {
    motor.stop();
  } else {
    motor.turn(rotate_cw, TURN_SPEED);
  }

  const unsigned long ms = 1000;
  unsigned long start = millis();

  while (!renew_log_file && millis() - start < ms) {
    if (ENABLE_GPS) {
      while (ss.available()) {
        gps.encode(ss.read());
      }
    }

    readMpu9250Value(buffer);

    if (digitalRead(flight_pin)) {
      motor.stop();
    }

    if (digitalRead(button)) {
      // スイッチが押されていない
      is_button_pressing = false;
    } else {
      // スイッチが押されている
      if (!is_button_pressing) {
        // スイッチが押されたときの1回目
        is_button_pressing = true;
        Serial.println("Create new log file");

        createNewLogFile();
        renew_log_file = true;

        rotate_cw = !rotate_cw;
        motor.stop();
      }
    }
  }

  if (!renew_log_file && ENABLE_GPS) {
    readGpsValue(buffer);
  }
  sd->appendFileString(SD, log_filename.c_str(), buffer);
}

static void createNewLogFile() {
  current_log_number++;
  sd->writeFileInt(SD, PREVIOUS_NUMBER_FILE, current_log_number);

  Serial.print("Next number: ");
  Serial.println(current_log_number);

  log_filename = String(LOG_DIR);
  log_filename += String("/");
  log_filename += String(current_log_number);
  log_filename += String(".csv");

  String message = "";

  if (ENABLE_GPS) {
    message += String("GPS,Testing TinyGPS++ library v. ");
    message += String(TinyGPSPlus::libraryVersion());
    message += String("\n");
    message += String("GPS,Sats,HDOP,Latitude,Longitude,Fix Age,Date,Time,DateAge,Alt,Course,Speed,Card,DistanceToG,CourseToG,CardToG,CharsRX,SentencesRX,ChecksumFail\n");
  }

  message += String("MPU9250,Yaw,Pitch,Roll,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,MyYaw\n");
  Serial.print(message);
  sd->appendFileString(SD, log_filename.c_str(), message);
}

static void readMpu9250Value(String& buffer) {
  String message = "MPU9250,";
  double my_yaw = 0.0;
  double mag_x = 0.0;
  double mag_y = 0.0;

  if (mpu.update()) {
    message += String(mpu.getYaw(), 6);
    message += String(",");
    message += String(mpu.getPitch(), 6);
    message += String(",");
    message += String(mpu.getRoll(), 6);
    message += String(",");

    message += String(mpu.getAccX(), 6);
    message += String(",");
    message += String(mpu.getAccY(), 6);
    message += String(",");
    message += String(mpu.getAccZ(), 6);
    message += String(",");
    message += String(mpu.getGyroX(), 6);
    message += String(",");
    message += String(mpu.getGyroY(), 6);
    message += String(",");
    message += String(mpu.getGyroZ(), 6);
    message += String(",");
    mag_x = mpu.getMagX();
    message += String(mag_x, 6);
    message += String(",");
    mag_y = mpu.getMagY();
    message += String(mag_y, 6);
    message += String(",");
    message += String(mpu.getMagZ(), 6);
    message += String(",");

    if (mag_x != 0.0 || mag_y != 0.0) {
      my_yaw = std::acos(mag_y / std::sqrt(mag_x * mag_x + mag_y * mag_y)) / PI * 180;
      if (mag_x < 0) {
        my_yaw *= -1;
      }
    }
    message += String(my_yaw, 6);

    message += String("\n");
    Serial.print(message);
    buffer += message;
  }
}

void readGpsValue(String& buffer) {
  String message = "GPS,";

  if (gps.satellites.isValid()) {
    message += String(gps.satellites.value());
  }
  message += String(",");

  if (gps.hdop.isValid()) {
    message += String(gps.hdop.hdop(), 6);
  }
  message += String(",");

  if (gps.hdop.isValid()) {
    message += String(gps.hdop.hdop(), 6);
  }
  message += String(",");

  if (gps.location.isValid()) {
    message += String(gps.location.lat(), 6);
  }
  message += String(",");

  if (gps.location.isValid()) {
    message += String(gps.location.lng(), 6);
  }
  message += String(",");

  if (gps.location.isValid()) {
    message += String(gps.location.age());
  }
  message += String(",");

  if (gps.date.isValid())
  {
    message += String(gps.date.year());
    message += String("-");
    message += String(gps.date.month());
    message += String("-");
    message += String(gps.date.day());
    message += String("T");
  }
  if (gps.time.isValid())
  {
    message += String(gps.time.hour());
    message += String(":");
    message += String(gps.time.minute());
    message += String(":");
    message += String(gps.time.second());
  }
  message += String(",");

  if (gps.date.isValid())
  {
    message += String(gps.date.age());
  }
  message += String(",");


  if (gps.altitude.isValid()) {
    message += String(gps.altitude.meters(), 6);
  }
  message += String(",");

  if (gps.course.isValid()) {
    message += String(gps.course.deg(), 6);
  }
  message += String(",");

  if (gps.speed.isValid()) {
    message += String(gps.speed.kmph(), 6);
  }
  message += String(",");

  if (gps.course.isValid()) {
    message += String(TinyGPSPlus::cardinal(gps.course.deg()));
  }
  message += String(",");

  unsigned long distanceKmToGoal =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      GOAL_LAT,
      GOAL_LON);

  if (gps.location.isValid()) {
    message += String(distanceKmToGoal, 6);
  }
  message += String(",");

  double courseToGoal =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      GOAL_LAT,
      GOAL_LON);

  if (gps.location.isValid()) {
    message += String(courseToGoal, 6);
  }
  message += String(",");

  const char *cardinalToGoal = TinyGPSPlus::cardinal(courseToGoal);

  if (gps.location.isValid()) {
    message += String(cardinalToGoal);
  }
  message += String(",");

  message += String(gps.charsProcessed());
  message += String(",");
  message += String(gps.sentencesWithFix());
  message += String(",");
  message += String(gps.failedChecksum());
  message += String("\n");

  Serial.print(message);
  buffer += message;
}
