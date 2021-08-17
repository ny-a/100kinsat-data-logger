#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "Arduino.h"

class Motor {
  public:
    Motor();

    void forward(int pwm); // 前進
    void back(int pwm); // 後退
    void turn(bool cw, int pwm);
    void move(int pwm_a, int pwm_b);
    void stop(); // 停止

  private:
    const int motorA[3] = {13, 4, 25}; // AIN1, AIN2, PWMA
    const int motorB[3] = {14, 27, 26}; // BIN1, BIN2, PWMB

    const int CHANNEL_A = 0;
    const int CHANNEL_B = 1;

    const int LEDC_TIMER_BIT = 8;
    const int LEDC_BASE_FREQ = 490;
};

#endif // __MOTOR_H__
