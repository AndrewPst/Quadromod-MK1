#include <MotionController.h>

MotionController &motionController = MotionController::getInstance();

PosParameters_t parameters;

int16_t MotionController::positions[LEGS_COUNT][2];
Leg *MotionController::legs_sort[LEGS_COUNT];
uint16_t MotionController::last_speed;
float MotionController::last_acc = 0;
MotionMode_t MotionController::_currentMotionMode = MotionMode_t::MODE_DYNAMIC;
StaticMotionPosition_t MotionController::_staticPosition = StaticMotionPosition_t::POS_LEFT_SPEARED;

#include <Actions.h>

void MotionController::initialize()
{
    for (uint8_t i = 0; i < LEGS_COUNT; i++)
    {
        legs_sort[legs[i].getLegPosition()] = &legs[i];
    }
}

MotionController &MotionController::getInstance()
{
    static MotionController controller;
    return controller;
}

void MotionController::setExecutableAction(Action_t action)
{
    if (_executableAction != action)
    {
        _executableAction = action;
        _actionTick = 0;
    }
}

ActionStatus_t MotionController::executeAction()
{
    if (_executableAction != Action_t::MOVE_UNKNOWN)
    {
        auto result = motionFunctions[(uint8_t)_executableAction](_actionTick++);
        if (result == ActionStatus_t::STATUS_ACTION_IS_FINISHED)
            _executableAction = Action_t::MOVE_UNKNOWN;
        return result;
    }
    else
        return ActionStatus_t::STATUS_ACTION_NOT_EXECUTED;
}

void MotionController::setMotionMode(MotionMode_t mode)
{
    _currentMotionMode = mode;
}

MotionMode_t MotionController::getMotionMode()
{
    return _currentMotionMode;
}

bool MotionController::setClearance(int16_t z)
{
    if (parameters.clearance == z)
        return false;
    parameters.clearance = z;
    for (uint8_t i = 0; i < LEGS_COUNT; i++)
    {
        legs[i].setPos(legs[i].getX(), legs[i].getY(), z);
    }
    return true;
}

int16_t MotionController::getLegLiftHeight()
{
    return constrain((double)parameters.clearance / 2.5, 0, parameters.clearance - BODY_HEIGHT);
}