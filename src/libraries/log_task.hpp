#pragma once

#include "sd_log.hpp"
#include "cansat_io.hpp"

// WebServer の有効化
#define ENABLE_WEB_SERVER false

#if ENABLE_WEB_SERVER == true
#include "web_controller.hpp"
#endif

#define SD_BUFFER_COUNT 128
#define QUEUE_BUFFER_SIZE 256

// WebServer 用 WiFi AP の接続情報
#define WIFI_SSID "Espressif-32CS"
#define WIFI_PASS "oOxzAnmt"

class LogTask {
  public:
    LogTask(SdLog * sdLog, CanSatIO * canSatIO);
    void sendToLoggerTask(String &buffer, bool skippable);
    void sendToLoggerTask(const char * buffer, bool skippable);
    void setupTask();
    void restartOnError();
    void restartOnError(int blinkCount);

    QueueHandle_t queue;
    SdLog * sdLog;
    char sdBuffer[SD_BUFFER_COUNT + 8][QUEUE_BUFFER_SIZE];
    int sdBufferPosition = 0;
    unsigned long sdLastWrite = 0;
    unsigned long sdWriteInterval = 2000;

  private:
    static void loggerTask(void *pvParameters);

    CanSatIO * canSatIO;
};

LogTask::LogTask(SdLog * sdLog, CanSatIO * canSatIO) {
  this->sdLog = sdLog;
  this->canSatIO = canSatIO;
}

void LogTask::sendToLoggerTask(String &buffer, bool skippable) {
  sendToLoggerTask(buffer.c_str(), skippable);
}

void LogTask::sendToLoggerTask(const char * buffer, bool skippable) {
  BaseType_t status;

  Serial.print(buffer);

  if (!skippable || uxQueueMessagesWaiting(queue) == 0) {
    status = xQueueSend(queue, buffer, 0);

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
      strcpy(thisPointer->sdBuffer[thisPointer->sdBufferPosition], buffer);
      thisPointer->sdBufferPosition++;

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

    if (0 < thisPointer->sdBufferPosition) {
      unsigned long now = millis();
      if (
        SD_BUFFER_COUNT < thisPointer->sdBufferPosition ||
        thisPointer->sdWriteInterval < now - thisPointer->sdLastWrite
      ) {
        for (int i = 0; i < thisPointer->sdBufferPosition; i++) {
          bool writeStatus = thisPointer->sdLog->writeLog(thisPointer->sdBuffer[i]);
          if (writeStatus) {
            writeFailedCount = 0;
          } else {
            Serial.println("SD append failed.");
            writeFailedCount++;
          }
        }
        thisPointer->sdBufferPosition = 0;
        thisPointer->sdLastWrite = now;
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
