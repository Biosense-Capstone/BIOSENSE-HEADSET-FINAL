#ifndef SPI_SCREEN_DRIVER
#define SPI_SCREEN_DRIVER
#include <stdint.h>
#include <cstring>
#include "spimaster.h"
#include "unistd.h"
#include <ti/drivers/GPIO.h>



void digitalWrite(uint8_t pin, uint8_t value);
uint8_t bitRead(uint32_t number, uint32_t bit);
uint8_t highByte(uint16_t word);
uint8_t lowByte(uint16_t word);
long map(long x, long in_min, long in_max, long out_min, long out_max);


void delay(uint32_t time);

class SPIClass
{
  public:
    //inline static void beginTransaction(SPISettings settings);
    inline static void endTransaction(void);
    inline static uint8_t transfer(uint8_t data);
    inline static uint16_t transfer16(uint16_t data);
    inline static void transfer(void *buf, size_t count);
    inline static void transmit(uint8_t data);
    inline static void transmit16(uint16_t data);
    inline static void transmit(void *buf, size_t count);

};
SPIClass SPI_Driver;

/*void SPIClass::beginTransaction(SPISettings settings)
{

}*/

void SPIClass::endTransaction(void)
{

}

uint8_t SPIClass::transfer(uint8_t data)
{
  return spi_send(data);
}

uint16_t SPIClass::transfer16(uint16_t data)
{

}

void SPIClass::transfer(void *buf, size_t count)
{

}

void SPIClass::transmit(uint8_t data)
{

}

void SPIClass::transmit16(uint16_t data)
{

}

void SPIClass::transmit(void *buf, size_t count)
{

}




#endif
