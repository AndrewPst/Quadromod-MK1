#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <setup.h>

enum CoordinateDirection_t : uint8_t
{
    DIR_NORMAL = 0,
    DIR_MIRROR = 1
};

enum LegPosition_t : uint8_t
{
    LEG_FORWARD_LEFT = 0,
    LEG_FORWARD_RIGHT,
    LEG_BACK_LEFT,
    LEG_BACK_RIGHT,
    LEG_NONE
};

#define ACC_AUTO_SET 0

class Leg
{
private:
    Servo _servo[SERVO_ON_LEG_COUNT];
    LegPosition_t _position = LEG_NONE;

    int16_t _currentPos[SERVO_ON_LEG_COUNT]{0};
    double _currentPosAngle[SERVO_ON_LEG_COUNT]{0};
    double _targetPos[SERVO_ON_LEG_COUNT]{0};
    double _accel[SERVO_ON_LEG_COUNT]{0};
    double _startPos[SERVO_ON_LEG_COUNT]{0};

    bool _isMotion = false;

    //uint32_t _timer = 0;
    uint16_t _motionCounter = 0;

    static float _acc_koef;
    static uint16_t _rotation_resolution;
public:
    Leg() = delete;
    Leg(LegPosition_t direction);

    void attach(uint8_t *pins);
    void goToHome();

    void tick();
    bool isMotion();

    static void setSpeed(uint16_t value);
    static uint16_t getSpeed();

    static void setAccel(float accel);
    static float getAccel();

    static void getMaxXandY(double z, double angle, int16_t& x, int16_t& y);
    void setPos(int16_t x, int16_t y, int16_t z);
    LegPosition_t getLegPosition();

    int16_t getX();
    int16_t getY();
    int16_t getZ();
};

extern Leg legs[LEGS_COUNT];