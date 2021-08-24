// Reference:
// https://github.com/espressif/arduino-esp32/tree/master/libraries/SD

# pragma once

/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */

#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"


#define LOG_DIR "/log/"
#define PREVIOUS_NUMBER_FILE "/prev_log_number.txt"

class SdLog {
  public:
    SdLog();

    bool openFileForAppend(fs::FS &fs, const char *path);
    void closeFile();
    bool writeLog(const char * message);
    bool createDirIfNotExist(fs::FS &fs, const char * dirname);

    int openNextLogFile(fs::FS &fs);

  private:
    String fileName;
    File file;
    bool isAvailable = false;
    uint8_t cardType;

    bool existDir(fs::FS &fs, const char * dirname);
    bool createDir(fs::FS &fs, const char * path);
};

/**
 * @brief SDカードの初期化処理
 *
 */
SdLog::SdLog() {
  if (!SD.begin()) {
    return;
  }
  cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    return;
  }

  isAvailable = true;
}

bool SdLog::openFileForAppend(fs::FS &fs, const char *path) {
  file = fs.open(path, FILE_APPEND);
  return (bool)file;
}

void SdLog::closeFile() {
  file.close();
}

/**
 * @brief ファイルに書き込む
 *
 * @param message
 */
bool SdLog::writeLog(const char * message) {
  if (!file) {
    return false;
  }
  return file.print(message) != 0;
}

int SdLog::openNextLogFile(fs::FS &fs) {
  File numberFile = fs.open(PREVIOUS_NUMBER_FILE);
  int number = 0;
  if (numberFile) {
    number = numberFile.parseInt() + 1;
    numberFile.close();
  }

  String message = String(number);

  numberFile = fs.open(PREVIOUS_NUMBER_FILE, FILE_WRITE);
  if (numberFile) {
    numberFile.print(message);
    numberFile.close();
  }

  fileName = LOG_DIR;
  fileName += message;
  fileName += String(".csv");

  createDirIfNotExist(fs, LOG_DIR);

  openFileForAppend(fs, fileName.c_str());

  return number;
}


bool SdLog::createDirIfNotExist(fs::FS &fs, const char * dirname) {
  return existDir(fs, dirname) || createDir(fs, dirname);
}

bool SdLog::existDir(fs::FS &fs, const char * dirname){
  File root = fs.open(dirname);
  return root && root.isDirectory();
}

bool SdLog::createDir(fs::FS &fs, const char * path){
  return fs.mkdir(path);
}