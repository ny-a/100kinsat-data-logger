#include "cansat_sd.hpp"
#include <TinyGPS++.h>
#include "MPU9250.h"
#include "motor.hpp"
#include <cmath>

static const bool ENABLE_GPS = true;
static const bool REDUCE_MPU_LOG = false;

static const int SPEED = 200;

// ゴールの緯度経度
static const double GOAL_LAT = 35.682716, GOAL_LON = 139.759955;

static const int YAW_DIFF_THRESHOLD = 3;

static const char* LOG_DIR = "/log";
static const char* PREVIOUS_NUMBER_FILE = "/prev_log_number.txt";

CanSatSd *sd;

static const int GPS_RX_Pin = 2;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial ss(GPS_RX_Pin);

MPU9250 mpu;

Motor motor = Motor();

#define LED 2
static const uint8_t flight_pin = 34;
static const uint8_t button = 35;

int current_log_number = 0;

bool is_button_pressing = false;
String log_filename = "";

int speed_a = 0;
int speed_b = 0;
bool rotate_cw = true;

double yaw = 0.0;
double pitch = 0.0;
double roll = 0.0;
double north_yaw = 0.0;
double target_yaw = 90;

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

  pinMode(LED, OUTPUT);
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

  if (digitalRead(flight_pin)) {
    // フライトピンが抜けている
    digitalWrite(LED, HIGH);
    Serial.println("Start Calibrate.");
    fastCalibrate();
    digitalWrite(LED, LOW);

    delay(1000);
  }

  digitalWrite(LED, HIGH);
  while (digitalRead(button)) {
    if (mpu.update()) {
      north_yaw = mpu.getYaw() + 180.0;
      Serial.println(north_yaw);
    }
  }
  digitalWrite(LED, LOW);
  is_button_pressing = true;

  Serial.print("north_yaw: ");
  Serial.println(north_yaw);

  sd->appendFileString(SD, log_filename.c_str(), String("north_yaw: ") + String(north_yaw, 6) + String("\n"));

  delay(1000);
}


void loop()
{
  bool renew_log_file = false;
  String buffer = "";

  if (digitalRead(flight_pin)) {
    speed_a = 0;
    speed_b = 0;
  } else {
    speed_a = SPEED;
    speed_b = SPEED;
  }

  const unsigned long ms = 1000;
  unsigned long start = millis();

  while (!renew_log_file && millis() - start < ms) {
    if (ENABLE_GPS) {
      while (ss.available()) {
        gps.encode(ss.read());
      }
    }

    readMpu9250Value(buffer, !REDUCE_MPU_LOG);

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

        // 反転させる
        if (target_yaw == 0) {
          target_yaw = 180;
        } else {
          target_yaw = 0;
        }
      }
    }

    if (
      digitalRead(flight_pin) ||
      pitch < -70 ||
      0 < pitch ||
      roll < -45 ||
      45 < roll
    ) {
      // フライトピンが抜かれるか、機体が横転したら止める
      speed_a = 0;
      speed_b = 0;
    } else {
      int diff = target_yaw - yaw;
      if (diff < -180) {
        diff += 360;
      } else if (180 < diff) {
        diff -= 360;
      }
      if (diff < -YAW_DIFF_THRESHOLD) {
        digitalWrite(LED, LOW);
        speed_a = SPEED + diff * 2;
        if (speed_a < -256) {
          speed_a = -256;
        }
        speed_b = SPEED;
      } else if (YAW_DIFF_THRESHOLD < diff) {
        digitalWrite(LED, LOW);
        speed_a = SPEED;
        speed_b = SPEED - diff * 2;
        if (speed_b < -256) {
          speed_b = -256;
        }
      } else {
        digitalWrite(LED, HIGH);
        speed_a = SPEED;
        speed_b = SPEED;
      }
    }
    motor.move(speed_a, speed_b);
  }

  if (REDUCE_MPU_LOG) {
    readMpu9250Value(buffer, true);
  }
  if (!renew_log_file && ENABLE_GPS) {
    readGpsValue(buffer);
  }
  sd->appendFileString(SD, log_filename.c_str(), buffer);
}

void fastCalibrate() {
  speed_a = SPEED * (rotate_cw ? 1 : -1);
  speed_b = SPEED * (rotate_cw ? -1 : 1);
  motor.move(speed_a, speed_b);

  const unsigned long ms = 10000;
  unsigned long start = millis();

  double mag_x_max = 0.0;
  double mag_x_min = 0.0;
  double mag_y_max = 0.0;
  double mag_y_min = 0.0;
  double mag_z_max = 0.0;
  double mag_z_min = 0.0;

  while (millis() - start < ms) {
    if (mpu.update()) {
      double mag_x = mpu.getMagX();
      double mag_y = mpu.getMagY();
      double mag_z = mpu.getMagZ();

      if (mag_x_max < mag_x) {
        mag_x_max = mag_x;
      }
      if (mag_x < mag_x_min) {
        mag_x_min = mag_x;
      }
      if (mag_y_max < mag_y) {
        mag_y_max = mag_y;
      }
      if (mag_y < mag_y_min) {
        mag_y_min = mag_y;
      }
      if (mag_z_max < mag_z) {
        mag_z_max = mag_z;
      }
      if (mag_z < mag_z_min) {
        mag_z_min = mag_z;
      }
    }


    if (!digitalRead(flight_pin)) {
      // フライトピンが刺されたら止める
      speed_a = 0;
      speed_b = 0;
      motor.move(speed_a, speed_b);
    }
  }

  speed_a = 0;
  speed_b = 0;
  motor.move(speed_a, speed_b);

  String message = "";

  message += String("mag_x_max: ");
  message += String(mag_x_max, 6);
  message += String("\n");
  message += String("mag_x_min: ");
  message += String(mag_x_min, 6);
  message += String("\n");
  message += String("mag_x_middle: ");
  double new_mag_x = mag_x_max - ((mag_x_max - mag_x_min) / 2);
  message += String(new_mag_x, 6);
  message += String("\n");
  message += String("mag_x_old: ");
  message += String(mpu.getMagBiasX(), 6);
  message += String("\n");
  message += String("mag_y_max: ");
  message += String(mag_y_max, 6);
  message += String("\n");
  message += String("mag_y_min: ");
  message += String(mag_y_min, 6);
  message += String("\n");
  message += String("mag_y_middle: ");
  double new_mag_y = mag_y_max - ((mag_y_max - mag_y_min) / 2);
  message += String(new_mag_y, 6);
  message += String("\n");
  message += String("mag_y_old: ");
  message += String(mpu.getMagBiasY(), 6);
  message += String("\n");
  message += String("mag_z_max: ");
  message += String(mag_z_max, 6);
  message += String("\n");
  message += String("mag_z_min: ");
  message += String(mag_z_min, 6);
  message += String("\n");
  message += String("mag_z_middle: ");
  double new_mag_z = mag_z_max - ((mag_z_max - mag_z_min) / 2);
  message += String(new_mag_z, 6);
  message += String("\n");
  message += String("mag_z_old: ");
  message += String(mpu.getMagBiasZ(), 6);
  message += String("\n");
  message += String("\n");
  message += String("mag_x_new: ");
  message += String(mpu.getMagBiasX() + new_mag_x, 6);
  message += String("\n");
  message += String("mag_y_new: ");
  message += String(mpu.getMagBiasY() + new_mag_y, 6);
  message += String("\n");
  message += String("mag_z_new: ");
  message += String(mpu.getMagBiasZ() + new_mag_z, 6);
  message += String("\n");

  Serial.println(message);
  sd->appendFileString(SD, log_filename.c_str(), message);

  mpu.setMagBias(
    mpu.getMagBiasX() + new_mag_x,
    mpu.getMagBiasY() + new_mag_y,
    mpu.getMagBiasZ() + new_mag_z
  );
}

void createNewLogFile() {
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

void readMpu9250Value(String& buffer, bool log) {
  String message = "MPU9250,";
  double my_yaw = 0.0;
  double mag_x = 0.0;
  double mag_y = 0.0;

  if (log || mpu.update()) {
    yaw = mpu.getYaw() + 180.0 - north_yaw;
    if (yaw < 0.0) {
      yaw += 360.0;
    }
    pitch = mpu.getPitch();
    roll = mpu.getRoll();
    message += String(yaw, 6);
    message += String(",");
    message += String(pitch, 6);
    message += String(",");
    message += String(roll, 6);
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
      my_yaw += 180.0;
    }
    message += String(my_yaw, 6);

    message += String("\n");
    if (log) {
      Serial.print(message);
      buffer += message;
    }
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

  unsigned int distanceKmToGoal =
    (unsigned int)TinyGPSPlus::distanceBetween(
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
    // ゴールへの方位に向ける
    // target_yaw = courseToGoal;
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
