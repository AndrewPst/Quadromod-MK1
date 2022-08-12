#pragma once

#include <Leg.h>

enum MotionMode_t : uint8_t
{
    MODE_UNKNOWN = 0,
    MODE_STATIC,
    MODE_DYNAMIC
};

enum Action_t : uint8_t
{
    MOVE_DYNAMIC = 0,
    MOVE_SET_DYNAMIC_MODE,
    MOVE_SET_STATIC_MODE,
    MOVE_STATIC_BACK_FORVARD,
    MOVE_STATIC_ROTATION,
    MOVE_OFFSET,
    MOVE_ROTATION,
    DANCE_1,
    DANCE_2,
    GIVE_THE_POW,
    MOVE_UNKNOWN
};

enum ActionStatus_t : uint8_t
{
    STATUS_UNKNOWN = 0,
    STATUS_ACTION_IS_RUNNING,
    STATUS_ACTION_IS_FINISHED,
    STATUS_ACTION_NOT_EXECUTED
};

enum StaticMotionPosition_t : uint8_t
{
    POS_LEFT_SPEARED = 0,
    POS_RIGHT_SPEARED
};

struct PosParameters_t
{
    int16_t legLiftHeight = LEG_LIFT_HEIGHT;

    int16_t rotationAngle = 0;
    float moveAngle = 0;
    int16_t moveSpeed = 0;

    int16_t clearance = HOME_POS_Z;

    int16_t xOffset;
    int16_t yOffset;
    int16_t zOffset;

    int16_t yRotation = 0;
    int16_t xRotation = 0;
    int16_t zRotation = 0;
};

extern PosParameters_t parameters;

class MotionController
{
    typedef ActionStatus_t (*MotionFunc_t)(uint16_t tick);

private:
    const MotionFunc_t motionFunctions[10] = {
        DynamicMotionFunc,
        SetDynamicModeFunc,
        SetStaticModeFunc,
        StaticMotion,
        StaticRotation,
        SetOffset,
        SetRotation,
        PlayDance1,
        PlayDance2,
        GiveThePaw,
    };

    MotionController() = default;
    MotionController(const MotionController &) = delete;
    MotionController &operator=(const MotionController &) = delete;

    Action_t _executableAction = Action_t::MOVE_UNKNOWN;
    uint16_t _actionTick = 0;

    static StaticMotionPosition_t _staticPosition;
    static MotionMode_t _currentMotionMode;
    static int16_t positions[LEGS_COUNT][2];
    static Leg *legs_sort[LEGS_COUNT];
    static uint16_t last_speed;
    static float last_acc;

public:
    static MotionController &getInstance();

    void initialize();

    void setExecutableAction(Action_t action);
    ActionStatus_t executeAction();

    void setMotionMode(MotionMode_t mode);
    MotionMode_t getMotionMode();

    bool setClearance(int16_t z);

    int16_t getLegLiftHeight();
    // void setXoffset(int16_t &x);
    // void setYoffset(int16_t &y);

    // void setRotationX(int16_t &x);
    // void setRotationY(int16_t &y);
    // void setRotationZ(int16_t &z);

private:
    static ActionStatus_t DynamicMotionFunc(uint16_t tick);
    static ActionStatus_t SetStaticModeFunc(uint16_t tick);
    static ActionStatus_t SetDynamicModeFunc(uint16_t tick);
    static ActionStatus_t StaticMotion(uint16_t tick);
    static ActionStatus_t StaticRotation(uint16_t tick);
    static ActionStatus_t SetOffset(uint16_t tick);
    static ActionStatus_t SetRotation(uint16_t tick);
    static ActionStatus_t PlayDance1(uint16_t tick);
    static ActionStatus_t PlayDance2(uint16_t tick);
    static ActionStatus_t GiveThePaw(uint16_t tick);
};

extern MotionController &motionController;