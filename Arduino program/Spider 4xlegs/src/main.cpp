#include <Arduino.h>
#include <Servo.h>
#include <Leg.h>

#include <setup.h>

#include <MotionController.h>
#include <Parser.h>

//Buffer size for parsing incoming data from serial
#define SERIAL_BUFFER_SIZE 64


//HEADERS
bool RotateAllServo();
void parseReceivedData();
void setMotionMode(MotionMode_t m);
void checkSerial();


//Global main variables
DataParser parser;
char serialBuffer[SERIAL_BUFFER_SIZE];
bool needUpdate = false;

bool isAllActionExecuted = true;
bool isAllServoRotated = false;
bool isCycleAction = false;

MotionMode_t lastMotionMode = MotionMode_t::MODE_DYNAMIC;

Action_t needExecuteAction = Action_t::MOVE_UNKNOWN;

void setup()
{
    Serial.begin(9600);
    Serial.setTimeout(8);

    //Debug
    Serial.println(F("Init"));


    //Initialize
    motionController.initialize();

    Leg::setSpeed(50);
    Leg::setAccel(ACC_AUTO_SET);
    //Set all servo rotation to 90* 
    for (uint8_t i = 0; i < LEGS_COUNT; i++)
    {
        legs[i].attach((uint8_t *)servo_pins[i]);
        legs[i].goToHome();
        delay(INIT_PAUSE);
    }
    while (!RotateAllServo())
        ;

    Leg::setSpeed(75);
}

void loop()
{
    if (!isAllServoRotated) //Если сервы не повернуты
    {
        isAllServoRotated = RotateAllServo();
    }
    else if (!isAllActionExecuted) // если действие недовыполнено
    {
        isAllActionExecuted = motionController.executeAction() == ActionStatus_t::STATUS_ACTION_IS_FINISHED; //выолняем следующее
        isAllServoRotated = isAllActionExecuted;
        if (isAllActionExecuted == true && isCycleAction == false) // если выполнилось и не зациклено, сбрасываем
            needExecuteAction = Action_t::MOVE_UNKNOWN;
        //delay(1000);
    }
    else
    {
        if (needUpdate) // если пришли данные
        {
            parser.parse(serialBuffer);     //парсим
            if (parser.getStr(0)[0] == '#') //если команда
            {
                parseReceivedData();
            }
            needUpdate = false;
        }
        if (needExecuteAction != Action_t::MOVE_UNKNOWN) //если имеется дейсвие выполняем
        {
            motionController.setExecutableAction(needExecuteAction);
            isAllActionExecuted = false;
        }
        // else
        // {
        //     needExecuteAction = Action_t::GIVE_THE_POW;
        // }
    }
    checkSerial();
}

void parseReceivedData()
{
    char key = parser.getStr(1)[0];        //берем флаг команды
    MotionMode_t newMode = lastMotionMode; //тип движения
    if (key == 'P')                        //если флаг - парамеры
    {
        Leg::setSpeed(parser.getInt(2));
        Leg::setAccel(parser.getFloat(3));
        motionController.setClearance(map(parser.getInt(4), 0, 100, BODY_HEIGHT, MAX_Z_OFFSET));
        needExecuteAction = Action_t::MOVE_UNKNOWN;
        isAllServoRotated = false;
    }
    else
    {
        if (key == 'D') //если динамическое движение
        {
            newMode = MotionMode_t::MODE_DYNAMIC;
            needExecuteAction = Action_t::MOVE_DYNAMIC;
            parameters.moveSpeed = parser.getInt(2);
            parameters.moveAngle = parser.getInt(3);
            parameters.rotationAngle = map(parser.getInt(4), -100, 100, -20, 20);
            isCycleAction = true;
        }
        else if (key == 'S') //если статическое
        {
            newMode = MotionMode_t::MODE_STATIC;
            char action = parser.getStr(2)[0];
            if (action == 'M') //если движение (move)
            {
                needExecuteAction = Action_t::MOVE_STATIC_BACK_FORVARD;
                parameters.moveSpeed = parser.getInt(3);
                parameters.rotationAngle = 0;
            }
            else if (action == 'R') //если вращение
            {
                needExecuteAction = Action_t::MOVE_STATIC_ROTATION;
                parameters.rotationAngle = parser.getInt(3);
                parameters.moveSpeed = 0;
            }
            isCycleAction = true;
        }
        else if (key == 'O') //если сдвиг по осям
        {
            newMode = MotionMode_t::MODE_DYNAMIC;
            parameters.xOffset = parser.getInt(2);
            parameters.yOffset = parser.getInt(3);
            parameters.zOffset = parser.getInt(4);
            needExecuteAction = Action_t::MOVE_OFFSET;
            parameters.moveSpeed = 1;
            isCycleAction = false;
        }
        else if (key == 'R') //если вращение по осям
        {
            newMode = MotionMode_t::MODE_DYNAMIC;
            parameters.yRotation = parser.getInt(2);
            parameters.xRotation = parser.getInt(3);
            parameters.zRotation = parser.getInt(4);
            needExecuteAction = Action_t::MOVE_ROTATION;
            parameters.moveSpeed = 1;
            isCycleAction = false;
        }
        else if (key == 'C')
        {
            if (!strcmp("D1", parser.getStr(2)))
            {
                newMode = MotionMode_t::MODE_DYNAMIC;
                needExecuteAction = Action_t::DANCE_1;
            }
            else if (!strcmp("D2", parser.getStr(2)))
            {
                //newMode = MotionMode_t::MODE_DYNAMIC;
                needExecuteAction = Action_t::DANCE_2;
            }
            else if (!strcmp("GP", parser.getStr(2)))
            {
                newMode = MotionMode_t::MODE_DYNAMIC;
                needExecuteAction = Action_t::GIVE_THE_POW;
            }
            isCycleAction = false;
            parameters.moveSpeed = 1;
        }
        if (newMode != lastMotionMode) //если нужно сменить режим хотьбы
        {
            setMotionMode(newMode);
            isAllServoRotated = false;
            lastMotionMode = newMode;
            isCycleAction = false;
        }
        else if (parameters.moveSpeed == 0 && parameters.rotationAngle == 0) //если движения нет - сбрасываем команду
            needExecuteAction = Action_t::MOVE_UNKNOWN;
    }
}

void setMotionMode(MotionMode_t m) //выбираем нужный режим походки
{
    if (m == MotionMode_t::MODE_DYNAMIC)
    {
        needExecuteAction = Action_t::MOVE_SET_DYNAMIC_MODE;
        motionController.setMotionMode(MotionMode_t::MODE_DYNAMIC);
    }
    else if (m == MotionMode_t::MODE_STATIC)
    {
        needExecuteAction = Action_t::MOVE_SET_STATIC_MODE;
        motionController.setMotionMode(MotionMode_t::MODE_STATIC);
    }
}

bool RotateAllServo() //крутим на 1 шаг все сервы на всех ногах
{
    bool result = true;
    for (auto &l : legs)
    {
        l.tick();
        result = l.isMotion() ? false : result;
    }
    return result;
}

void checkSerial()
{
    if (Serial.available())
    {
        Serial.readBytes(serialBuffer, SERIAL_BUFFER_SIZE);
        needUpdate = true;
    }
}