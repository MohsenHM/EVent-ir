#ifndef EVENT_MOTOR_H
#define EVENT_MOTOR_H

#include <Arduino.h>
#include <avr/wdt.h>
#include <configuration.h>

// #define INITIAL_POLARITY LOW

#define DIRECTION_CLOSE         LOW
#define DIRECTION_OPEN          HIGH

#define MOTOR_IS_ON             HIGH
#define MOTOR_IS_OFF            LOW

#define MOTOR_PULSE_PER_TURN    1150

#define MOTOR_ENC_PERIOD_OFF    100000

#define RPM_AVG_N               8

#define MAXIMUM_MOTOR_SPEED_IN_RPM 70
#define MINIUM_MOTOR_SPEED_IN_RPM  4
#define MINIUM_MOTOR_SPEED_IN_PWM  27
#define REQURIED_SPEED             20
#define TIME_STEP                  5e-3
#define DESIRED_ROTATION           33
#define EXHALE_DEGREE_RATIO        0.9
#define BEFORE_OUSWITCH_MAX_DEGREE 15
#define MOTOR_STOP_TIME            35e-3

#define PID_IGNORE_COUNT           20
#define PID_GAURD_COUNT            20

class Motor
{
private:
    static Motor *INSTANCE;
    int motorSpeed = 245;
    int motorStatus = MOTOR_IS_OFF;
    int direction = DIRECTION_OPEN;

    long  encLastTime = 0;
    long  encLastCheck = 0;
    int   encPeriod = 0;
    int   encLastState = LOW;
    int   encDebounceTime = 1;
    int   encPulseCount = 0;
    int   rpmIndex=0;
    int   PC=0; //Pulse Counter
    float oldRPM=0;
    float RPMs[RPM_AVG_N];


public:
    static Motor *getInstance();
    Motor();
    void initEnc(int pin, uint8_t ioMode, void (*callback_func)(void), int interruptMode);
    int  getStatus();
    void changeDirection();
    void setDirection(int direction);
    int  getDirection();
    void setMotorOut();
    void setSpeed(int a);
    void setEncPeriod(int encPeriod);
    void incrementPC();
    void resetPC();
    int  getSpeed();
    int  getSpeedPWM();
    void motorStop();
    void motorStart();
    void motorSwitch();
    void encCallback();
    int  getEncCount();
    void resetEncPeriod();
    void resetEncRPM();
    float getEncRPM();
    int  getEncAngle();
    int  getEncPeriod();
    int  getPC();
};

#endif
