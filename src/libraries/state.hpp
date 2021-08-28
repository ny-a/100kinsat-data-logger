#pragma once

class State {
  public:
    State();
    void getLogString(String& buffer);
    void setGoal(double lat, double lng);
    static void userRequestReboot(void * pvParameters);
    static void copyGoal(void * pvParameters);
    static void copyTargetYaw(void * pvParameters);
    static void copyNorthYaw(void * pvParameters);

    bool targetIsGoal = true;
    double goalLat = 0.0;
    double goalLong = 0.0;
    double targetYaw = 0.0;
    double northYaw = 0.0;
    int motorLeft = 0;
    int motorRight = 0;
    int defaultSpeed = 255;
    int currentSpeed = 255;

    bool enableSdLog = true;

    bool rebootByUserRequest = false;
};

State::State() {
}

void State::getLogString(String& buffer) {
  buffer += "State,";
  buffer += targetIsGoal ? "targetIsGoal," : "goStraight,";
  buffer += String(goalLat, 6);
  buffer += String(",");
  buffer += String(goalLong, 6);
  buffer += String(",");
  buffer += String(targetYaw, 6);
  buffer += String(",");
  buffer += String(northYaw, 6);
  buffer += String(",");
  buffer += String(motorLeft);
  buffer += String(",");
  buffer += String(motorRight);
  buffer += String(",");
  buffer += String(currentSpeed);
  buffer += String("\n");
}

void State::setGoal(double lat, double lng) {
  this->goalLat = lat;
  this->goalLong = lng;
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
  thisPointer->targetIsGoal = otherPointer->targetIsGoal;
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
  thisPointer->targetIsGoal = otherPointer->targetIsGoal;
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
