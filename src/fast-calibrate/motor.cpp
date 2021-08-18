#include "motor.hpp"

Motor::Motor() {
  for (int i = 0; i < 3; i++) {
    pinMode(motorA[i], OUTPUT);
    pinMode(motorB[i], OUTPUT);
  }

  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ, LEDC_TIMER_BIT);

  ledcAttachPin(motorA[2], CHANNEL_A);
  ledcAttachPin(motorB[2], CHANNEL_B);
}

/**
 * 前進
 */
void Motor::forward(int pwm) {
  // 左モータ（CCW，反時計回り）
  digitalWrite(motorA[1], LOW);
  digitalWrite(motorA[0], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CW，時計回り）
  digitalWrite(motorB[1], LOW);
  digitalWrite(motorB[0], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/**
 * 後退
 */
void Motor::back(int pwm) {
  // 左モータ（CW，時計回り）
  digitalWrite(motorA[0], LOW);
  digitalWrite(motorA[1], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CCW，反時計回り）
  digitalWrite(motorB[0], LOW);
  digitalWrite(motorB[1], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}


void Motor::turn(bool cw, int pwm) {
  // 左モータ（CW，時計回り）
  digitalWrite(motorA[(int) cw], LOW);
  digitalWrite(motorA[(int)!cw], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CW，時計回り）
  digitalWrite(motorB[(int)!cw], LOW);
  digitalWrite(motorB[(int) cw], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}


void Motor::move(int pwm_a, int pwm_b) {
  // 左モータ
  if (pwm_a == 0) {
    digitalWrite(motorA[0], LOW);
    digitalWrite(motorA[1], LOW);
    ledcWrite(CHANNEL_A, HIGH);
  } else {
    bool cw_a = pwm_a < 0;
    digitalWrite(motorA[(int)!cw_a], LOW);
    digitalWrite(motorA[(int) cw_a], HIGH);
    ledcWrite(CHANNEL_A, std::abs(pwm_a));
  }

  // 右モータ
  if (pwm_b == 0) {
    digitalWrite(motorB[0], LOW);
    digitalWrite(motorB[1], LOW);
    ledcWrite(CHANNEL_B, HIGH);
  } else {
    bool cw_b = pwm_b < 0;
    digitalWrite(motorB[(int)!cw_b], LOW);
    digitalWrite(motorB[(int) cw_b], HIGH);
    ledcWrite(CHANNEL_B, std::abs(pwm_b));
  }
}

/**
 * 停止
 */
void Motor::stop() {
  // 左モータ停止
  digitalWrite(motorA[0], LOW);
  digitalWrite(motorA[1], LOW);
  ledcWrite(CHANNEL_A, HIGH);

  // 右モータ停止
  digitalWrite(motorB[0], LOW);
  digitalWrite(motorB[1], LOW);
  ledcWrite(CHANNEL_B, HIGH);
}
