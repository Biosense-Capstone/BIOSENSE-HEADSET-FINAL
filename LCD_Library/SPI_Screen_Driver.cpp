#include "SPI_Screen_Driver.h"


void digitalWrite(uint8_t pin, uint8_t value)
{
    GPIO_write(pin, value);
}

void delay(uint32_t time)
{
    int  i;
    for(i=0;i<time*1000;i++);
    //usleep(time * 1000);
}

uint8_t bitRead(uint32_t number, uint32_t bit)
{
    return ((number >> bit) & 0b1);
}

uint8_t highByte(uint16_t word)
{
    return ((word >> 8) & 0xFF);
}

uint8_t lowByte(uint16_t word)
{
    return (word & 0xFF);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
