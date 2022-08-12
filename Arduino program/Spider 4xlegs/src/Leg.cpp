#include <Leg.h>

Leg legs[LEGS_COUNT] = {
    Leg(LegPosition_t::LEG_FORWARD_LEFT),
    Leg(LegPosition_t::LEG_FORWARD_RIGHT),
    Leg(LegPosition_t::LEG_BACK_LEFT),
    Leg(LegPosition_t::LEG_BACK_RIGHT),
};

float Leg::_acc_koef = 1;
uint16_t Leg::_rotation_resolution = DEFAULT_ROTATION_RESOLUTION;

//Индексы серв в массиве
#define SERVO_1 0
#define SERVO_2 1
#define SERVO_3 2

#define LENGHT_2_POW LENGHT_2 *LENGHT_2
#define LENGHT_3_POW LENGHT_3 *LENGHT_3

double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Leg::Leg(LegPosition_t direction)
{
    _position = direction;
}

void Leg::attach(uint8_t *pins)
{
    for (uint8_t i = 0; i < SERVO_ON_LEG_COUNT; i++)
    {
        _servo[i].attach(pins[i]);
        _currentPosAngle[i] = 90;
        _servo[i].write(_currentPosAngle[i]);
    }
}

void Leg::goToHome()
{
    setPos(HOME_POS_X, HOME_POS_Y, HOME_POS_Z);
}

void Leg::tick()
{
    if (!_isMotion)
        return;
    for (uint8_t i = 0; i < SERVO_ON_LEG_COUNT; i++)
    {
        //используем формулу x = x0 + V0*t+(a*t^2)/2; Также изменияем под условия
        if (_motionCounter <= _rotation_resolution / 2)
            _currentPosAngle[i] = _startPos[i] + _accel[i] * pow(_motionCounter, _acc_koef);
        else
            _currentPosAngle[i] = _targetPos[i] - _accel[i] * (pow(_rotation_resolution - _motionCounter, _acc_koef));
        _servo[i].write(_currentPosAngle[i]);
    }
    if (_motionCounter >= _rotation_resolution)
    {
        _isMotion = false;
    }
    _motionCounter++;
    // _timer = millis();
}

bool Leg::isMotion()
{
    return _isMotion;
}

/*
    Установка необходимой позиции для ноги в координатной системе в мм;
*/
void Leg::setPos(int16_t x, int16_t y, int16_t z)
{
    //Добро пожаловать в ад для микроконтроллера
    // Serial.println("===");
    // Serial.println(_position);
    _currentPos[0] = x;
    _currentPos[1] = y;
    _currentPos[2] = z;

    // Serial.print(x);
    // Serial.print('/');
    // Serial.print(y);
    // Serial.print('/');
    // Serial.println(z);

    //позиция в градусах для 1 части ноги
    //формула alfa = (arctan(<противолежащий катет>/<прилежащий катет>)*180)/PI
    // часть <значение>*180 / PI переводит из радиан в градусы
    // 90- - т.к. y=0 при угле 90*
    _targetPos[SERVO_1] = 90 - degrees(atan((double)y / (double)x));

    //гипотенуза между X и Y осями - длина 1 части, т.к. она не участвует в посроении
    /*
                                        ^
               Zzzzzz-------------------|Y
               |     zzzzz              |
               |          zzzzz         |
               |               LLLLL    |
            <------------------------LLLO - - - - - - - 
            X
        Где:
            --- - оси
            X, Y - название осей
            ZZZ - temp_hypot
            LLL - длина 1 части (LENGHT_1)
    */
    double temp_hypot = sqrt(pow(x, 2) + pow(y, 2)) - LENGHT_1;
    // Serial.print(temp_hypot);
    // Serial.print('/');
    //угол для поворота 2 части ноги
    //Если Z < 0 формулка другая, т.к. значения переворачиваются
    double temp_angle;
    if (z >= 0)
        temp_angle = degrees(atan(temp_hypot / (double)z));
    else
        temp_angle = 90 + degrees(atan((double)-z / temp_hypot));

    //Расстояние между нужной точкой и осью 2 ноги (по Пифагору)
    temp_hypot = sqrt(pow(temp_hypot, 2) + pow(z, 2));
    // Serial.print(temp_hypot);
    // Serial.println('/');
    temp_hypot = constrain(temp_hypot, max(LENGHT_3, LENGHT_2) - min(LENGHT_3, LENGHT_2), temp_hypot);

    //временная переменная для формулы
    double temp_hypot_pow = temp_hypot * temp_hypot;

    //по теореме косинусов
    _targetPos[SERVO_2] = temp_angle + degrees(acos((LENGHT_2_POW + temp_hypot_pow - LENGHT_3_POW) / (2.0 * temp_hypot * LENGHT_2)));
    _targetPos[SERVO_3] = 180.0 - degrees(acos((LENGHT_3_POW + LENGHT_2_POW - temp_hypot_pow) / (2.0 * LENGHT_3 * LENGHT_2)));

    // Serial.print(_targetPos[SERVO_1]);
    // Serial.print('/');
    // Serial.print(_targetPos[SERVO_2]);
    // Serial.print('/');
    // Serial.println(_targetPos[SERVO_3]);

    for (uint8_t i = 0; i < SERVO_ON_LEG_COUNT; i++)
    {
        //Если ноги отзеркалены, меняем направление
        if (_position == LegPosition_t::LEG_FORWARD_RIGHT || _position == LegPosition_t::LEG_BACK_LEFT)
        {
            _targetPos[i] = constrain(180 - _targetPos[i], 180 - max_servo_angle[i][1], 180 - max_servo_angle[i][0]);
        }
        else
        {
            _targetPos[i] = constrain(_targetPos[i], max_servo_angle[i][0], max_servo_angle[i][1]);
        }

        //высчитываем ускорение по формуле a = (2*S)/t^2; Изменяем формулу под половину расстояния и времени с учетом торможения
        _accel[i] = ((_targetPos[i] - _currentPosAngle[i]) / 2) / (pow(_rotation_resolution / 2, _acc_koef));
        _startPos[i] = _currentPosAngle[i];
    }
    _isMotion = true;
    _motionCounter = 1;
}

int16_t Leg::getX() { return _currentPos[0]; }
int16_t Leg::getY() { return _currentPos[1]; }
int16_t Leg::getZ() { return _currentPos[2]; }

LegPosition_t Leg::getLegPosition()
{
    return _position;
}

void Leg::setSpeed(uint16_t value)
{
    _rotation_resolution = constrain(map(value, 100, 0, MIN_ROTAION_RESOLUTION, MAX_ROTATION_RESOLUTION), MIN_ROTAION_RESOLUTION, MAX_ROTATION_RESOLUTION);
}

uint16_t Leg::getSpeed()
{
    return map(_rotation_resolution, MIN_ROTAION_RESOLUTION, MAX_ROTATION_RESOLUTION, 100, 0);
}

void Leg::setAccel(float accel)
{
    if (accel == ACC_AUTO_SET)
    {
        _acc_koef = mapf(_rotation_resolution, MAX_ROTATION_RESOLUTION, MIN_ROTAION_RESOLUTION, 1.0, 2.0);
    }
    else
        _acc_koef = accel < 1 ? 1 : accel;
}

float Leg::getAccel()
{
    return _acc_koef;
}

void Leg::getMaxXandY(double z, double angle, int16_t &x, int16_t &y)
{
    double hypot_lenght = sqrt(pow(LENGHT_2 + LENGHT_3, 2) - pow((double)z, 2)) + LENGHT_1;
    x = sin(radians(angle) + M_PI_2) * hypot_lenght;
    y = cos(radians(angle) + M_PI_2) * hypot_lenght;
}