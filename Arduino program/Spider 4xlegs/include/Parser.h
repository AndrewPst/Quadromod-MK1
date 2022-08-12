#ifndef ReceivedData_h
#define ReceivedData_h

#include <Arduino.h>

#define DIVIDER '|'

class DataParser
{
public:
    // передать char array строку, можно указать символ разделитель
    DataParser() = default;
    ~DataParser();

    // разделить строку на подстроки
    uint16_t parse(char *buf);

    uint16_t getLenght();

    // получить инт из выбранной подстроки
    int16_t getInt(int num);

    // получить float из выбранной подстроки
    float getFloat(int num);

    char *getStr(uint16_t idx);

    char *operator[](uint16_t idx);

private:
    // количество разделённых данных в пакете
    uint16_t amount(char* buf);

    // освободить буфер
    void clear();

    uint16_t lenght_ = 0;

    char **str_ = NULL;
};

#endif