#include <Parser.h>

DataParser::~DataParser()
{
    clear();
}

void DataParser::clear()
{

    if (str_)
        free(str_);

}

uint16_t DataParser::amount(char *buf)
{
    uint16_t i = 0, count = 0;

    while (buf[i++])
        if (buf[i] == DIVIDER)
            count++; // подсчёт разделителей
    lenght_ = ++count;

    return lenght_;
}

uint16_t DataParser::getLenght()
{

    auto value = lenght_;

    return value;
}

uint16_t DataParser::parse(char *buf)
{
    clear();                   // освобождаем буфер
    uint16_t am = amount(buf); // количество данных
    str_ = (char **)malloc(am * sizeof(char *)); // создаём буфер
    str_[0] = buf;                               // строка 0
    int i = 0, j = 0;                            // счётчики
    while (buf[i])
    { // пока не NULL
        if (buf[i] == DIVIDER)
        {                            // если разделитель
            buf[i] = '\0';           // меняем на NULL
            str_[++j] = buf + i + 1; // запоминаем начало строки
        }
        i++;
    }

    return am;
}

int16_t DataParser::getInt(int num)
{

    auto value = atol(str_[num]);

    return value;
}

float DataParser::getFloat(int num)
{

    auto value = atof(str_[num]);

    return value;
}

char *DataParser::getStr(uint16_t idx)
{
    if (idx >= lenght_)
        return nullptr;
    auto value = str_[idx];
    return value;
}

char *DataParser::operator[](uint16_t idx)
{
    return getStr(idx);
}
