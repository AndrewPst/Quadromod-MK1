#pragma once

//==================Параметры ног================
#define SERVO_ON_LEG_COUNT 3
#define LEGS_COUNT 4

//================размеры корпуса(в мм)==========
#define LENGHT_1 72.5 // 1 часть
#define LENGHT_2 84.0 //2 часть
#define LENGHT_3 149.5 //3 часть

//размеры тела
#define BODY_LENGHT 145.4
#define BODY_WIDTH 135.0
#define BODY_HEIGHT 40


//======параметры движения========
//скорость обновления позиции сервы. Чем больше - тем плавнее и медленней
#define MIN_ROTAION_RESOLUTION 48
#define DEFAULT_ROTATION_RESOLUTION 128
#define MAX_ROTATION_RESOLUTION (256 + 128) 

#define MAX_ROTATION_ANGLE ((25 * M_PI) / 180.0) //угол вращения вокруг осей

#define HOME_POS_Y ((LENGHT_1 + LENGHT_2) / 2)
#define HOME_POS_X ((LENGHT_1 + LENGHT_2 + LENGHT_3) / 2.5)
#define HOME_POS_Z BODY_HEIGHT * 3

#define MOTION_RADIUS (HOME_POS_Y)

#define LEG_LIFT_HEIGHT (HOME_POS_Z / 2)
#define LEG_LOWERING_HEIGHT 0

#define INIT_PAUSE 300

#define MAX_Z_OFFSET (BODY_HEIGHT + (LENGHT_2 + LENGHT_3) / 2)

#define MAX_HYPOT_LENGHT LENGHT_2 + LENGHT_3


//==================Подключение============
static constexpr int16_t max_servo_angle[LEGS_COUNT][SERVO_ON_LEG_COUNT]{
    {0, 135},
    {35, 180},
    {0, 150},
};

static constexpr uint8_t servo_pins[LEGS_COUNT][SERVO_ON_LEG_COUNT]{
    {7, 5, 6},
    {11, 12, 13},
    {3, 2, 4},
    {9, 10, 8},
};
