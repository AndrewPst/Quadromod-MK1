#include <setup.h>

ActionStatus_t MotionController::DynamicMotionFunc(uint16_t tick)
{
    switch (tick)
    {
    case 0:
    {
        float motion_radius = MOTION_RADIUS;
        motion_radius = parameters.moveSpeed / 100.0 * MOTION_RADIUS;
        last_speed = Leg::getSpeed();
        last_acc = Leg::getAccel();
        double angle = (parameters.moveAngle * M_PI) / 180.0;

        double posXFromC = HOME_POS_X + BODY_WIDTH / 2;
        double posYFromC = HOME_POS_Y + BODY_LENGHT / 2;
        double lenght_to_pos = sqrt(posYFromC * posYFromC + posXFromC * posXFromC);
        double angle1 = (atan((HOME_POS_Y + (BODY_LENGHT / 2)) / (HOME_POS_X + (BODY_WIDTH / 2))) * 180) / M_PI + parameters.rotationAngle;

        for (uint8_t i = 0; i < LEGS_COUNT; i++)
        {
            int16_t posX, posY;
            auto v = legs[i].getLegPosition();
            if (v == LegPosition_t::LEG_FORWARD_LEFT)
            {
                posX = motion_radius * sin(angle) * 0.8;
                posY = motion_radius * cos(angle);
            }
            else if (v == LegPosition_t::LEG_FORWARD_RIGHT)
            {
                posX = motion_radius * cos(angle - M_PI_2) * 0.8;
                posY = motion_radius * sin(angle - M_PI_2);
            }
            else if (v == LegPosition_t::LEG_BACK_LEFT)
            {
                posX = motion_radius * cos(angle + M_PI_2) * 0.8;
                posY = motion_radius * sin(angle + M_PI_2);
            }
            else if (v == LegPosition_t::LEG_BACK_RIGHT)
            {
                posX = motion_radius * sin(angle - M_PI) * 0.8;
                posY = motion_radius * cos(angle - M_PI);
            }
            double angle = ((i % 3 != 0) ? 180.0 - angle1 : angle1);
            positions[i][0] = abs(lenght_to_pos * cos((angle * M_PI) / 180)) - BODY_WIDTH / 2 + posX;
            positions[i][1] = lenght_to_pos * sin((angle * M_PI) / 180) - BODY_LENGHT / 2 + posY;
        }
        Leg::setSpeed(100);
        Leg::setAccel(1.3);
        legs_sort[0]->setPos((HOME_POS_X + positions[0][0]) / 2, (HOME_POS_Y + positions[0][1]) / 2, parameters.clearance - motionController.getLegLiftHeight());
        legs_sort[3]->setPos((HOME_POS_X + positions[3][0]) / 2, (HOME_POS_Y + positions[3][1]) / 2, parameters.clearance - motionController.getLegLiftHeight());
        legs_sort[1]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance + LEG_LOWERING_HEIGHT);
        legs_sort[2]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance + LEG_LOWERING_HEIGHT);
        break;
    }
    case 1:
        Leg::setAccel(last_acc);
        legs_sort[0]->setPos(positions[0][0], positions[0][1], parameters.clearance);
        legs_sort[3]->setPos(positions[3][0], positions[3][1], parameters.clearance);
        legs_sort[1]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[2]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        break;
    case 2:
        Leg::setSpeed(last_speed);
        legs_sort[0]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[3]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[1]->setPos(positions[1][0], positions[1][1], parameters.clearance);
        legs_sort[2]->setPos(positions[2][0], positions[2][1], parameters.clearance);
        break;
    case 3:
        Leg::setSpeed(100);
        Leg::setAccel(1.3);
        legs_sort[0]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance + LEG_LOWERING_HEIGHT);
        legs_sort[3]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance + LEG_LOWERING_HEIGHT);
        legs_sort[1]->setPos((HOME_POS_X + positions[1][0]) / 2, (HOME_POS_Y + positions[1][1]) / 2, parameters.clearance - motionController.getLegLiftHeight());
        legs_sort[2]->setPos((HOME_POS_X + positions[2][0]) / 2, (HOME_POS_Y + positions[2][1]) / 2, parameters.clearance - motionController.getLegLiftHeight());
        break;
    case 4:
        Leg::setAccel(last_acc);
        legs_sort[0]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[3]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[1]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[2]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        break;
    default:
        Leg::setSpeed(last_speed);
        return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
        break;
    }
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}

static uint8_t legs_index_left_speared[LEGS_COUNT]{1, 0, 3, 2};
static uint8_t legs_index_right_speared[LEGS_COUNT]{0, 1, 2, 3};
static uint8_t legs_index_left_speared_back[LEGS_COUNT]{3, 2, 1, 0};
static uint8_t legs_index_right_speared_back[LEGS_COUNT]{2, 3, 0, 1};

uint8_t *selected_index_arr = NULL;

ActionStatus_t MotionController::SetDynamicModeFunc(uint16_t tick)
{
    switch (tick)
    {
    case 0:
    {
        if (_staticPosition == StaticMotionPosition_t::POS_LEFT_SPEARED)
            selected_index_arr = legs_index_left_speared;
        else if (_staticPosition == StaticMotionPosition_t::POS_RIGHT_SPEARED)
            selected_index_arr = legs_index_right_speared;
        legs_sort[selected_index_arr[0]]->setPos(HOME_POS_X, HOME_POS_Y + MOTION_RADIUS / 2, BODY_HEIGHT);
        legs_sort[selected_index_arr[3]]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY() - MOTION_RADIUS / 2, parameters.clearance);
        legs_sort[selected_index_arr[1]]->setPos(legs_sort[1]->getX(), legs_sort[1]->getY() + MOTION_RADIUS / 2, parameters.clearance);
        legs_sort[selected_index_arr[2]]->setPos(legs_sort[2]->getX(), legs_sort[2]->getY() - MOTION_RADIUS / 2, parameters.clearance);
        break;
    }
    case 1:
        legs_sort[selected_index_arr[0]]->setPos(HOME_POS_X, HOME_POS_Y + MOTION_RADIUS / 2, parameters.clearance);
        break;
    case 2:
        legs_sort[selected_index_arr[0]]->setPos(legs_sort[0]->getX(), legs_sort[0]->getY() - MOTION_RADIUS, parameters.clearance);
        legs_sort[selected_index_arr[1]]->setPos(legs_sort[1]->getX(), legs_sort[1]->getY() - MOTION_RADIUS, parameters.clearance);
        legs_sort[selected_index_arr[2]]->setPos(HOME_POS_X, HOME_POS_Y + MOTION_RADIUS / 2, BODY_HEIGHT);
        legs_sort[selected_index_arr[3]]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY() + MOTION_RADIUS, parameters.clearance);
        break;
    case 3:
        legs_sort[selected_index_arr[2]]->setPos(HOME_POS_X, HOME_POS_Y + MOTION_RADIUS / 2, parameters.clearance);
        break;
    case 4:
        legs_sort[selected_index_arr[0]]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[selected_index_arr[1]]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[selected_index_arr[2]]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[selected_index_arr[3]]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        break;
    default:
        return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
        break;
    }
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}

ActionStatus_t MotionController::SetStaticModeFunc(uint16_t tick)
{
    switch (tick)
    {
    case 0:
        _staticPosition = StaticMotionPosition_t::POS_RIGHT_SPEARED;
        legs_sort[0]->setPos(HOME_POS_X, MOTION_RADIUS / 2, BODY_HEIGHT);
        legs_sort[3]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY() - MOTION_RADIUS / 2, parameters.clearance);
        legs_sort[1]->setPos(legs_sort[1]->getX(), legs_sort[1]->getY() + MOTION_RADIUS / 2, parameters.clearance);
        legs_sort[2]->setPos(legs_sort[2]->getX(), legs_sort[2]->getY() - MOTION_RADIUS / 2, parameters.clearance);
        break;
    case 1:
        legs_sort[0]->setPos(HOME_POS_X, MOTION_RADIUS / 2, parameters.clearance);
        break;
    case 2:
        legs_sort[2]->setPos(HOME_POS_X, MOTION_RADIUS / 2, BODY_HEIGHT);
        legs_sort[0]->setPos(legs_sort[0]->getX(), legs_sort[0]->getY() - MOTION_RADIUS, parameters.clearance);
        legs_sort[3]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY() + MOTION_RADIUS, parameters.clearance);
        legs_sort[1]->setPos(legs_sort[1]->getX(), legs_sort[1]->getY() - MOTION_RADIUS, parameters.clearance);
        break;
    case 3:
        legs_sort[2]->setPos(HOME_POS_X, MOTION_RADIUS / 2, parameters.clearance);
        break;
    case 4:
        legs_sort[0]->setPos(legs_sort[0]->getX(), 0, parameters.clearance);
        legs_sort[1]->setPos(legs_sort[1]->getX(), HOME_POS_Y, parameters.clearance);
        legs_sort[2]->setPos(legs_sort[2]->getX(), 0, parameters.clearance);
        legs_sort[3]->setPos(legs_sort[3]->getX(), HOME_POS_Y, parameters.clearance);
        break;
    default:
        return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
        break;
    }
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}

ActionStatus_t MotionController::StaticMotion(uint16_t tick)
{
    switch (tick)
    {
    case 0:
    {
        if (_staticPosition == StaticMotionPosition_t::POS_LEFT_SPEARED && parameters.moveSpeed > 0)
            selected_index_arr = legs_index_left_speared;
        else if (_staticPosition == StaticMotionPosition_t::POS_RIGHT_SPEARED && parameters.moveSpeed > 0)
            selected_index_arr = legs_index_right_speared;
        else if (_staticPosition == StaticMotionPosition_t::POS_LEFT_SPEARED && parameters.moveSpeed < 0)
            selected_index_arr = legs_index_left_speared_back;
        else if (_staticPosition == StaticMotionPosition_t::POS_RIGHT_SPEARED && parameters.moveSpeed < 0)
            selected_index_arr = legs_index_right_speared_back;
        last_speed = Leg::getSpeed();
        last_acc = Leg::getAccel();
        Leg::setSpeed(85);
        Leg::setAccel(1.2);
        legs_sort[selected_index_arr[0]]->setPos(HOME_POS_X, (legs_sort[selected_index_arr[0]]->getY() + HOME_POS_Y + MOTION_RADIUS) / 2, parameters.clearance - motionController.getLegLiftHeight());
        break;
    }
    case 1:
        Leg::setAccel(last_acc);
        legs_sort[selected_index_arr[0]]->setPos(HOME_POS_X, HOME_POS_Y + MOTION_RADIUS, parameters.clearance);
        break;
    case 2:
        Leg::setSpeed(last_speed);
        legs_sort[selected_index_arr[0]]->setPos(HOME_POS_X, legs_sort[selected_index_arr[0]]->getY() - MOTION_RADIUS, parameters.clearance);
        legs_sort[selected_index_arr[1]]->setPos(HOME_POS_X, legs_sort[selected_index_arr[1]]->getY() - MOTION_RADIUS, parameters.clearance);
        legs_sort[selected_index_arr[2]]->setPos(HOME_POS_X, legs_sort[selected_index_arr[2]]->getY() + MOTION_RADIUS, parameters.clearance);
        legs_sort[selected_index_arr[3]]->setPos(HOME_POS_X, legs_sort[selected_index_arr[3]]->getY() + MOTION_RADIUS, parameters.clearance);
        break;
    case 3:
        Leg::setSpeed(85);
        Leg::setAccel(1.2);
        legs_sort[selected_index_arr[3]]->setPos(HOME_POS_X, (HOME_POS_Y + MOTION_RADIUS + HOME_POS_Y - MOTION_RADIUS) / 2, parameters.clearance - motionController.getLegLiftHeight());
        break;
    case 4:
        Leg::setAccel(last_acc);
        legs_sort[selected_index_arr[3]]->setPos(HOME_POS_X, HOME_POS_Y - MOTION_RADIUS, parameters.clearance);
        break;
    default:
        Leg::setSpeed(last_speed);
        _staticPosition = (_staticPosition == StaticMotionPosition_t::POS_LEFT_SPEARED) ? StaticMotionPosition_t::POS_RIGHT_SPEARED : StaticMotionPosition_t::POS_LEFT_SPEARED;
        return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
    }
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}

ActionStatus_t MotionController::StaticRotation(uint16_t tick)
{
    switch (tick)
    {
    case 0:
    {
        if (_staticPosition == StaticMotionPosition_t::POS_LEFT_SPEARED && parameters.rotationAngle < 0)
            selected_index_arr = legs_index_left_speared;
        else if (_staticPosition == StaticMotionPosition_t::POS_RIGHT_SPEARED && parameters.rotationAngle > 0)
            selected_index_arr = legs_index_right_speared;
        else if (_staticPosition == StaticMotionPosition_t::POS_LEFT_SPEARED && parameters.rotationAngle > 0)
            selected_index_arr = legs_index_left_speared_back;
        else if (_staticPosition == StaticMotionPosition_t::POS_RIGHT_SPEARED && parameters.rotationAngle < 0)
            selected_index_arr = legs_index_right_speared_back;
        //setSelectedIndexArr(_staticPosition, parameters.rotationAngle);
        double newX = HOME_POS_X + BODY_WIDTH / 2;
        double newY = HOME_POS_Y / 2;
        double angle = atan2(newY, newX);
        double lenght = sqrt(newX * newX + newY * newY);
        positions[1][0] = lenght * cos(0) - BODY_WIDTH / 2;
        positions[1][1] = lenght * sin(0) + HOME_POS_Y / 2;

        newX = HOME_POS_X + BODY_WIDTH / 2;
        newY = HOME_POS_Y + BODY_LENGHT + HOME_POS_Y / 2;
        lenght = sqrt(newX * newX + newY * newY);
        angle = (acos(newX / lenght) + angle);
        positions[0][0] = lenght * cos(angle) - BODY_WIDTH / 2;
        positions[0][1] = abs(lenght * sin(angle)) - BODY_LENGHT - HOME_POS_Y / 2;

        legs_sort[selected_index_arr[0]]->setPos(legs_sort[selected_index_arr[0]]->getX(), legs_sort[selected_index_arr[0]]->getY(), parameters.clearance - motionController.getLegLiftHeight());
        break;
    }
    case 1:
        legs_sort[selected_index_arr[0]]->setPos(positions[0][0], positions[0][1], parameters.clearance - motionController.getLegLiftHeight());
        legs_sort[selected_index_arr[1]]->setPos(positions[0][0], positions[0][1], parameters.clearance);
        legs_sort[selected_index_arr[2]]->setPos(positions[1][0], positions[1][1], parameters.clearance);
        legs_sort[selected_index_arr[3]]->setPos(positions[1][0], positions[1][1], parameters.clearance);
        break;
    case 2:
        legs_sort[selected_index_arr[0]]->setPos(positions[0][0], positions[0][1], parameters.clearance);
        break;
    case 3:
        legs_sort[selected_index_arr[1]]->setPos(positions[0][0], positions[0][1], parameters.clearance - motionController.getLegLiftHeight());
        break;
    case 4:
        legs_sort[selected_index_arr[0]]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[selected_index_arr[1]]->setPos(HOME_POS_X, HOME_POS_Y - MOTION_RADIUS, parameters.clearance - motionController.getLegLiftHeight());
        legs_sort[selected_index_arr[2]]->setPos(HOME_POS_X, HOME_POS_Y, parameters.clearance);
        legs_sort[selected_index_arr[3]]->setPos(HOME_POS_X, HOME_POS_Y - MOTION_RADIUS, parameters.clearance);
        break;
    case 5:
        legs_sort[selected_index_arr[1]]->setPos(HOME_POS_X, HOME_POS_Y - MOTION_RADIUS, parameters.clearance);
        break;
    case 6:
        _staticPosition = (_staticPosition == StaticMotionPosition_t::POS_LEFT_SPEARED) ? StaticMotionPosition_t::POS_RIGHT_SPEARED : StaticMotionPosition_t::POS_LEFT_SPEARED;
        return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
    }
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}

ActionStatus_t MotionController::SetOffset(uint16_t tick)
{
    if (tick == 0)
    {
        int16_t xo = (double)parameters.xOffset / 100.0 * MOTION_RADIUS;
        int16_t yo = (double)parameters.yOffset / 100.0 * MOTION_RADIUS;
        int16_t zo = constrain(((double)parameters.zOffset / 100.0 * MOTION_RADIUS + parameters.clearance), BODY_HEIGHT, MAX_Z_OFFSET);
        legs_sort[0]->setPos(HOME_POS_X + xo, HOME_POS_Y - yo, zo);
        legs_sort[1]->setPos(HOME_POS_X - xo, HOME_POS_Y - yo, zo);
        legs_sort[2]->setPos(HOME_POS_X + xo, HOME_POS_Y + yo, zo);
        legs_sort[3]->setPos(HOME_POS_X - xo, HOME_POS_Y + yo, zo);
        return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
    }
    return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
}

ActionStatus_t MotionController::SetRotation(uint16_t tick)
{
    if (tick > 0)
        return ActionStatus_t::STATUS_ACTION_IS_FINISHED;

    double rX = (double)parameters.xRotation / 100.0 * MAX_ROTATION_ANGLE;
    double rY = (double)parameters.yRotation / 100.0 * MAX_ROTATION_ANGLE;
    double rZ = (double)parameters.zRotation / 100.0 * MAX_ROTATION_ANGLE;

    double arr[4][3]{
        {-BODY_WIDTH / 2 - HOME_POS_X, BODY_LENGHT / 2 + HOME_POS_Y, (double)parameters.clearance},
        {BODY_WIDTH / 2 + HOME_POS_X, BODY_LENGHT / 2 + HOME_POS_Y, (double)parameters.clearance},
        {-BODY_WIDTH / 2 - HOME_POS_X, -BODY_LENGHT / 2 - HOME_POS_Y, (double)parameters.clearance},
        {BODY_WIDTH / 2 + HOME_POS_X, -BODY_LENGHT / 2 - HOME_POS_Y, (double)parameters.clearance},
    };

    for (uint8_t i = 0; i < 4; i++)
    {
        double x, y, z;
        y = arr[i][1] * cos(rX) + arr[i][2] * sin(rX);
        z = -arr[i][1] * sin(rX) + arr[i][2] * cos(rX);
        arr[i][1] = y;
        arr[i][2] = z;
        x = arr[i][0] * cos(rY) + arr[i][2] * sin(rY);
        z = -arr[i][0] * sin(rY) + arr[i][2] * cos(rY);
        arr[i][0] = x;
        arr[i][2] = z;
        x = arr[i][0] * cos(rZ) + arr[i][1] * sin(rZ);
        y = -arr[i][0] * sin(rZ) + arr[i][1] * cos(rZ);
        arr[i][0] = x;
        arr[i][1] = y;
    }
    legs_sort[0]->setPos((arr[0][0] * -1) - BODY_WIDTH / 2, arr[0][1] - BODY_LENGHT / 2, arr[0][2]);
    legs_sort[1]->setPos(arr[1][0] - BODY_WIDTH / 2, arr[1][1] - BODY_LENGHT / 2, arr[1][2]);
    legs_sort[2]->setPos((arr[2][0] * -1) - BODY_WIDTH / 2, (arr[2][1] * -1) - BODY_LENGHT / 2, arr[2][2]);
    legs_sort[3]->setPos(arr[3][0] - BODY_WIDTH / 2, (arr[3][1] * -1) - BODY_LENGHT / 2, arr[3][2]);
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}

#define DANCE_COUNT 3

ActionStatus_t MotionController::PlayDance1(uint16_t tick)
{
    if (tick == 0)
    {
        last_speed = Leg::getSpeed();
        last_acc = Leg::getAccel();
        legs_sort[0]->setPos(legs_sort[0]->getX(), HOME_POS_Y - MOTION_RADIUS / 2, HOME_POS_Z);
        legs_sort[1]->setPos(legs_sort[1]->getX(), HOME_POS_Y - MOTION_RADIUS / 2, HOME_POS_Z);
        legs_sort[2]->setPos(legs_sort[2]->getX(), HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z - motionController.getLegLiftHeight());
        legs_sort[3]->setPos(legs_sort[3]->getX(), HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z + motionController.getLegLiftHeight());
    }
    else if (tick <= 4 * DANCE_COUNT)
    {
        tick = (tick - 1) % 4;
        switch (tick)
        {
        case 0:
            legs_sort[2]->setPos(0, HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z + motionController.getLegLiftHeight());
            legs_sort[3]->setPos(legs_sort[3]->getX(), HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z + motionController.getLegLiftHeight());
            break;
        case 1:
        {
            Leg::setSpeed(60);
            Leg::setAccel(1.6);
            int16_t x, y;
            legs_sort[3]->getMaxXandY(-HOME_POS_Z / 2, degrees(atan2(HOME_POS_Y, LENGHT_1 + LENGHT_2 + LENGHT_3)), x, y);
            legs_sort[3]->setPos(x, y, -HOME_POS_Z / 2);
            legs_sort[2]->setPos(legs_sort[2]->getX(), legs_sort[2]->getY(), HOME_POS_Z - motionController.getLegLiftHeight());
            break;
        }
        case 2:
            legs_sort[2]->setPos(legs_sort[2]->getX(), legs_sort[2]->getY(), HOME_POS_Z + motionController.getLegLiftHeight());
            legs_sort[3]->setPos(0, HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z + motionController.getLegLiftHeight());
            break;
        case 3:
        {
            Leg::setSpeed(60);
            Leg::setAccel(1.6);
            int16_t x, y;
            legs_sort[2]->getMaxXandY(-HOME_POS_Z / 2, degrees(atan2(HOME_POS_Y, LENGHT_1 + LENGHT_2 + LENGHT_3)), x, y);
            legs_sort[2]->setPos(x, y, -HOME_POS_Z / 2);
            legs_sort[3]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY(), HOME_POS_Z - motionController.getLegLiftHeight());
            break;
        }
        default:
            break;
        }
    }
    else
    {
        tick = tick - 1 - 4 * DANCE_COUNT;
        switch (tick)
        {
        case 0:
            legs_sort[3]->setPos(legs_sort[3]->getX(), HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z + motionController.getLegLiftHeight());
            legs_sort[2]->setPos(HOME_POS_X, HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z + motionController.getLegLiftHeight());
            break;
        case 1:
            legs_sort[3]->setPos(HOME_POS_X, HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z - motionController.getLegLiftHeight());
            break;
        case 2:
            legs_sort[3]->setPos(HOME_POS_X, HOME_POS_Y + MOTION_RADIUS / 2, HOME_POS_Z + motionController.getLegLiftHeight());
            break;
        case 3:
            Leg::setSpeed(last_speed);
            Leg::setAccel(last_acc);
            legs_sort[0]->setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
            legs_sort[1]->setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
            legs_sort[2]->setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
            legs_sort[3]->setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
            break;
        default:
            return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
            break;
        }
    }
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}

ActionStatus_t MotionController::PlayDance2(uint16_t tick)
{
    switch (tick)
    {
    case 0:
        last_speed = Leg::getSpeed();
        Leg::setSpeed(80);
        legs_sort[1]->setPos(legs_sort[1]->getX(), legs_sort[1]->getY(), legs_sort[1]->getZ() - motionController.getLegLiftHeight());
        break;
    case 1:
        legs_sort[1]->setPos(legs_sort[1]->getX(), legs_sort[1]->getY(), legs_sort[1]->getZ() + motionController.getLegLiftHeight());
        break;
    case 2:
        legs_sort[3]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY(), legs_sort[3]->getZ() - motionController.getLegLiftHeight());
        break;
    case 3:
        legs_sort[3]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY(), legs_sort[3]->getZ() + motionController.getLegLiftHeight());
        break;
    case 4:
        legs_sort[2]->setPos(legs_sort[2]->getX(), legs_sort[2]->getY(), legs_sort[2]->getZ() - motionController.getLegLiftHeight());
        break;
    case 5:
        legs_sort[2]->setPos(legs_sort[2]->getX(), legs_sort[2]->getY(), legs_sort[2]->getZ() + motionController.getLegLiftHeight());
        break;
    case 6:
        legs_sort[0]->setPos(legs_sort[0]->getX(), legs_sort[0]->getY(), legs_sort[0]->getZ() - motionController.getLegLiftHeight());
        break;
    case 7:
        legs_sort[0]->setPos(legs_sort[0]->getX(), legs_sort[0]->getY(), legs_sort[0]->getZ() + motionController.getLegLiftHeight());
        break;
    case 8:
        legs_sort[1]->setPos(legs_sort[1]->getX(), legs_sort[1]->getY(), legs_sort[1]->getZ() + motionController.getLegLiftHeight());
        legs_sort[3]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY(), legs_sort[3]->getZ() + motionController.getLegLiftHeight());
        break;
    case 9:
        legs_sort[1]->setPos(legs_sort[1]->getX(), legs_sort[1]->getY(), legs_sort[1]->getZ() - motionController.getLegLiftHeight());
        legs_sort[3]->setPos(legs_sort[3]->getX(), legs_sort[3]->getY(), legs_sort[3]->getZ() - motionController.getLegLiftHeight());
        legs_sort[0]->setPos(legs_sort[0]->getX(), legs_sort[0]->getY(), legs_sort[0]->getZ() + motionController.getLegLiftHeight());
        legs_sort[2]->setPos(legs_sort[2]->getX(), legs_sort[2]->getY(), legs_sort[2]->getZ() + motionController.getLegLiftHeight());
        break;
    case 10:
        legs_sort[0]->setPos(legs_sort[0]->getX(), legs_sort[0]->getY(), legs_sort[0]->getZ() - motionController.getLegLiftHeight());
        legs_sort[2]->setPos(legs_sort[2]->getX(), legs_sort[2]->getY(), legs_sort[2]->getZ() - motionController.getLegLiftHeight());
        break;
    default:
        Leg::setSpeed(last_speed);
        return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
    }
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}

ActionStatus_t MotionController::GiveThePaw(uint16_t tick)
{
    static int16_t x, y;
    switch (tick)
    {
    case 0:
        last_speed = Leg::getSpeed();
        Leg::setSpeed(50);
        legs_sort[0]->getMaxXandY(10, degrees(atan2(LENGHT_1 + LENGHT_2 + LENGHT_3, -LENGHT_1 / 2)), x, y);
        legs_sort[0]->setPos(x, y, 10);
        legs_sort[1]->setPos(legs_sort[1]->getX(), HOME_POS_Y + MOTION_RADIUS , HOME_POS_Z + motionController.getLegLiftHeight());
        legs_sort[2]->setPos(legs_sort[2]->getX(), HOME_POS_Y - MOTION_RADIUS , HOME_POS_Z);
        legs_sort[3]->setPos(legs_sort[3]->getX(), HOME_POS_Y - MOTION_RADIUS , HOME_POS_Z);
        // Serial.print(x);
        // Serial.print('/');
        // Serial.println(y);
        break;
    case 1:
        // Serial.print(legs_sort[0]->getX());
        // Serial.print('/');
        // Serial.println(legs_sort[0]->getY() - (MOTION_RADIUS / 2));
        legs_sort[0]->setPos(legs_sort[0]->getX(), legs_sort[0]->getY() + (MOTION_RADIUS / 2), 10);
        break;
    case 2:
        legs_sort[0]->setPos(x, y, 10);
        break;
    case 3:
        legs_sort[0]->setPos(legs_sort[0]->getX(), legs_sort[0]->getY() + (MOTION_RADIUS / 2), 10);
        break;
    case 4:
        legs_sort[0]->setPos(x, y, 10);
        break;
    case 5:
        legs_sort[0]->setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
        legs_sort[1]->setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
        legs_sort[2]->setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
        legs_sort[3]->setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
        break;
    default:
        Leg::setSpeed(last_speed);
        return ActionStatus_t::STATUS_ACTION_IS_FINISHED;
        break;
    }
    return ActionStatus_t::STATUS_ACTION_IS_RUNNING;
}
