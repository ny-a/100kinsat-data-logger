#pragma once

class State {
  public:
    State();
    void getLogString(String& buffer);
    void setGoal(double lat, double lng);

    bool targetIsGoal = true;
    double goalLat = 0.0;
    double goalLong = 0.0;
    double targetYaw = 0.0;
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
