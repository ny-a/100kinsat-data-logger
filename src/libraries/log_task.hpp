#pragma once

#include "sd_log.hpp"
#include "cansat_io.hpp"

// WebServer の有効化
#define ENABLE_WEB_SERVER false

#if ENABLE_WEB_SERVER == true
#include "web_controller.hpp"
#endif

#define QUEUE_BUFFER_SIZE 256

// WebServer 用 WiFi AP の接続情報
#define WIFI_SSID "Espressif-32CS"
#define WIFI_PASS "oOxzAnmt"

class LogTask {
  public:
    LogTask(SdLog * sdLog, CanSatIO * canSatIO);
    void sendToLoggerTask(String &buffer, bool skippable);
    void setupTask();
    void restartOnError();
    void restartOnError(int blinkCount);

    QueueHandle_t queue;
    SdLog * sdLog;

  private:
    static void loggerTask(void *pvParameters);

    CanSatIO * canSatIO;
};

LogTask::LogTask(SdLog * sdLog, CanSatIO * canSatIO) {
  this->sdLog = sdLog;
  this->canSatIO = canSatIO;
}

void LogTask::sendToLoggerTask(String &buffer, bool skippable) {
  BaseType_t status;

  if (!skippable || uxQueueMessagesWaiting(queue) == 0) {
    status = xQueueSend(queue, buffer.c_str(), 0);

    if (status != pdPASS) {
      Serial.println("rtos queue send failed");
    }
  }
}

void LogTask::setupTask() {
  queue = xQueueCreate(64, QUEUE_BUFFER_SIZE);

  if(queue != NULL) {
    xTaskCreatePinnedToCore(this->loggerTask, "loggerTask", 4096, this, 1, NULL, 1);
  } else {
    Serial.println("rtos queue create error, stopped");
    restartOnError();
  }
}

void LogTask::loggerTask(void *pvParameters) {
  LogTask *thisPointer = (LogTask *) pvParameters;
  BaseType_t status;
  int writeFailedCount = 0;
  char buffer[QUEUE_BUFFER_SIZE];
  const TickType_t tick = 10U; // [ms]

  #if ENABLE_WEB_SERVER == true
  WebController webController(WIFI_SSID, WIFI_PASS);
  #endif

  while (true) {
    status = xQueueReceive(thisPointer->queue, buffer, tick);
    if(status == pdPASS) {
      Serial.print(buffer);
      bool writeStatus = thisPointer->sdLog->writeLog(buffer);
      if (writeStatus) {
        writeFailedCount = 0;
      } else {
        Serial.println("SD append failed.");
        writeFailedCount++;
      }

      #if ENABLE_WEB_SERVER == true
      if (buffer[0] == 'G') {
        webController.setValue(0, buffer);
      }
      if (buffer[0] == 'M') {
        webController.setValue(1, buffer);
      }
      #endif
    } else {
      if (uxQueueMessagesWaiting(thisPointer->queue) != 0) {
        Serial.println("rtos queue receive error?");
      }
    }
    if (10 < writeFailedCount) {
      Serial.println("sd write failed, stopped");
      thisPointer->restartOnError();
    }

    #if ENABLE_WEB_SERVER == true
    webController.handleClient();
    #endif
  }
}

void LogTask::restartOnError() {
  restartOnError(10);
}

void LogTask::restartOnError(int blinkCount) {
  unsigned long start = millis();
  unsigned long lastChanged = 0;
  bool isLEDOn = false;
  int i = 0;
  while (i < blinkCount * 2) {
    if (200 < millis() - lastChanged) {
      lastChanged = millis();
      isLEDOn = !isLEDOn;
      i++;
    }
    if (isLEDOn) {
      canSatIO->setLEDOn();
    } else {
      canSatIO->setLEDOff();
    }
  }
  Serial.println("Restarting...");
  delay(100);
  ESP.restart();
}
