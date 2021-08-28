#pragma once

enum class VehicleMode {
  Mission,
  FollowYaw,
  CalibrateMag,
  Stop,
  Completed,
};

class State {
  public:
    State();
    void getHeader(String& buffer);
    void getLogString(String& buffer);
    void setGoal(double lat, double lng);
    void checkYawForGpsCompensation();
    void doGpsCompensation();
    static void userRequestReboot(void * pvParameters);
    static void copyGoal(void * pvParameters);
    static void copyTargetYaw(void * pvParameters);
    static void copyNorthYaw(void * pvParameters);
    static void magCalibrate(void * pvParameters);
    static void stop(void * pvParameters);
    static void copySdLogState(void * pvParameters);

    static double clipYawDiff(double diff);
    static double clipYaw(double yaw);

    double goalDistance = 0.0;
    double goalLat = 0.0;
    double goalLong = 0.0;
    double targetYaw = 0.0;
    double northYaw = 0.0;
    double gpsYawDiff = 0.0;
    double yawDiff = 0.0;
    int motorLeft = 0;
    int motorRight = 0;
    int defaultSpeed = 255;
    int currentSpeed = 255;
    VehicleMode vehicleMode = VehicleMode::Mission;
    double magXMax = 0.0;
    double magYMax = 0.0;
    double magZMax = 0.0;
    double magXMin = 0.0;
    double magYMin = 0.0;
    double magZMin = 0.0;
    bool flightPinInserted = false;
    bool detectFallDown = false;
    int logFileNumber = 0;
    double yawDiffThreshold = 3.0;
    double gpsYawCompensationDiffLimit = 10.0;
    int gpsCompensationInterval = 5000;
    double gpsYawCompensationLimitAtOnce = 20.0;
    double gpsCompensationFactor = 3.0;

    unsigned long lastGpsYawCompensation = 0;
    double gpsYawCompensationSum = 0.0;
    int gpsYawCompensationItems = 0;
    double latestGpsDiff = 0.0;

    bool enableSdLog = true;

    bool rebootByUserRequest = false;
};

State::State() {
}

void State::getHeader(String& buffer) {
  buffer += "State,VehicleMode,yawDiff,gpsYawDiff,goalDistance,goalLat,goalLong,targetYaw,northYaw,GPSCItems,lastGPSC,motorLeft,motorRight,curentSpeed,flightPin,fallDown,millis,logFileNumber,sdLog,magX,magY,magZ\n";
}

void State::getLogString(String& buffer) {
  buffer += "State,";
  switch (vehicleMode)
  {
  case VehicleMode::Mission:
    buffer += "Mission,";
    break;
  case VehicleMode::FollowYaw:
    buffer += "FollowYaw,";
    break;
  case VehicleMode::CalibrateMag:
    buffer += "CalibrateMag,";
    break;
  case VehicleMode::Stop:
    buffer += "Stop,";
    break;
  case VehicleMode::Completed:
    buffer += "Completed,";
    break;
  default:
    buffer += "InvalidMode,";
    break;
  }
  buffer += String(yawDiff, 6);
  buffer += String(",");
  buffer += String(gpsYawDiff, 6);
  buffer += String(",");
  buffer += String(goalDistance, 6);
  buffer += String(",");
  buffer += String(goalLat, 6);
  buffer += String(",");
  buffer += String(goalLong, 6);
  buffer += String(",");
  buffer += String(targetYaw, 6);
  buffer += String(",");
  buffer += String(northYaw, 6);
  buffer += String(",");
  buffer += String(gpsYawCompensationItems);
  buffer += String(",");
  buffer += String(latestGpsDiff, 6);
  buffer += String(",");
  buffer += String(motorLeft);
  buffer += String(",");
  buffer += String(motorRight);
  buffer += String(",");
  buffer += String(currentSpeed);
  buffer += String(",");
  buffer += flightPinInserted ? "Inserted" : "Removed";
  buffer += String(",");
  buffer += detectFallDown ? "FallDown" : "Normal";
  buffer += String(",");
  buffer += String(static_cast<int>(millis() % 100000000));
  buffer += String(",");
  buffer += String(logFileNumber);
  buffer += String(",");
  buffer += enableSdLog ? "Enabled" : "Disabled";

  if (vehicleMode == VehicleMode::CalibrateMag) {
    buffer += String(", ");
    double magX = ((magXMax + magXMin) / 2);
    buffer += String(magX, 6);
    buffer += String(", ");
    double magY = ((magYMax + magYMin) / 2);
    buffer += String(magY, 6);
    buffer += String(", ");
    double magZ = ((magZMax + magZMin) / 2);
    buffer += String(magZ, 6);
  }
  buffer += String("\n");
}

void State::setGoal(double lat, double lng) {
  this->goalLat = lat;
  this->goalLong = lng;
}

void State::checkYawForGpsCompensation() {
  if (gpsYawCompensationDiffLimit < std::abs(yawDiff)) {
    lastGpsYawCompensation = millis();
    gpsYawCompensationSum = 0.0;
    gpsYawCompensationItems = 0;
  }
}

void State::doGpsCompensation() {
  if (gpsCompensationInterval < millis() - lastGpsYawCompensation) {
    if (gpsYawCompensationItems == 0) {
      lastGpsYawCompensation = millis();
    } else {
      latestGpsDiff = gpsYawCompensationSum / (gpsYawCompensationItems * gpsCompensationFactor);
      double clippedGpsDiff = latestGpsDiff;
      if (clippedGpsDiff < -gpsYawCompensationLimitAtOnce) {
        clippedGpsDiff = -gpsYawCompensationLimitAtOnce;
      } else if (gpsYawCompensationLimitAtOnce < clippedGpsDiff) {
        clippedGpsDiff = gpsYawCompensationLimitAtOnce;
      }
      northYaw = clipYaw(northYaw + clippedGpsDiff);
      lastGpsYawCompensation = millis();
      gpsYawCompensationSum = 0.0;
      gpsYawCompensationItems = 0;
    }
  } else {
    gpsYawCompensationSum += gpsYawDiff;
    gpsYawCompensationItems++;
  }
}

void State::userRequestReboot(void * pvParameters) {
  State *thisPointer = (State *) pvParameters;
  thisPointer->rebootByUserRequest = true;
  while (true) {}
}

void State::copyGoal(void * pvParameters) {
  State ** args = (State **) pvParameters;
  State * thisPointer = args[0];
  State * otherPointer = args[1];
  if (otherPointer->goalLat != 0.0) {
    thisPointer->goalLat = otherPointer->goalLat;
  }
  if (otherPointer->goalLong != 0.0) {
    thisPointer->goalLong = otherPointer->goalLong;
  }
  thisPointer->vehicleMode = VehicleMode::Mission;
  while (true) {
    delay(1);
  }
}

void State::copyTargetYaw(void * pvParameters) {
  State ** args = (State **) pvParameters;
  State * thisPointer = args[0];
  State * otherPointer = args[1];
  thisPointer->targetYaw = otherPointer->targetYaw;
  thisPointer->currentSpeed = otherPointer->currentSpeed;
  thisPointer->vehicleMode = VehicleMode::FollowYaw;
  while (true) {
    delay(1);
  }
}

void State::copyNorthYaw(void * pvParameters) {
  State ** args = (State **) pvParameters;
  State * thisPointer = args[0];
  State * otherPointer = args[1];
  thisPointer->northYaw = otherPointer->northYaw;
  while (true) {
    delay(1);
  }
}

void State::magCalibrate(void * pvParameters) {
  State ** args = (State **) pvParameters;
  State * thisPointer = args[0];
  thisPointer->vehicleMode = VehicleMode::CalibrateMag;
  while (true) {
    delay(1);
  }
}

void State::stop(void * pvParameters) {
  State ** args = (State **) pvParameters;
  State * thisPointer = args[0];
  thisPointer->vehicleMode = VehicleMode::Stop;
  while (true) {
    delay(1);
  }
}

void State::copySdLogState(void * pvParameters) {
  State ** args = (State **) pvParameters;
  State * thisPointer = args[0];
  thisPointer->vehicleMode = VehicleMode::Stop;
  State * otherPointer = args[1];
  thisPointer->enableSdLog = otherPointer->enableSdLog;
  while (true) {
    delay(1);
  }
}

double State::clipYawDiff(double diff) {
  if (diff < -180.0) {
    diff += 360.0;
  } else if (180.0 <= diff) {
    diff -= 360.0;
  }
  return diff;
}

double State::clipYaw(double yaw) {
  if (yaw < 0.0) {
    yaw += 360.0;
  } else if (360.0 <= yaw) {
    yaw -= 360.0;
  }
  return yaw;
}
