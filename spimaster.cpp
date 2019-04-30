/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== spimaster.c ========
 */
#define KALMAN

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <algorithm>
/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>


/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"

#include "spimaster.h"
#include "LCD_Library/Screen_HX8353E.h"
#ifdef KALMAN
#include "KalmanFilter-1.0.2/Kalman.h"
#endif
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Seconds.h>
#include "math.h"



#define THREADSTACKSIZE (1024)

#define SPI_MSG_LENGTH  (25)

#define MAX_LOOP        (10)

/* BMP Registers */
#define BMP_TEMP_XLSB_REG 0xFC
#define BMP_TEMP_LSB_REG 0xFB
#define BMP_TEMP_MSB_REG 0xFA
#define BMP_PRESS_XLSB_REG 0xF9
#define BMP_PRESS_LSB_REG 0xF8
#define BMP_PRESS_MSB_REG 0xF7
#define BMP_CONFIG_REG 0xF5
#define BMP_CTRL_MEAS_REG 0xF4
#define BMP_STATUS_REG 0xF3
#define BMP_RESET_REG 0xE0
#define BMP_ID_REG 0xD0
#define BMP_DIG_T1_LSB_REG 0x88
#define BMP_DIG_T1_MSB_REG 0x89
#define BMP_DIG_T2_LSB_REG 0x8A
#define BMP_DIG_T2_MSB_REG 0x8B
#define BMP_DIG_T3_LSB_REG 0x8C
#define BMP_DIG_T3_MSB_REG 0x8D

/* MPU Mag Registers */
#define MPU_MAG_ID_REG             0x00
#define MPU_MAG_INFO_REG           0x01
#define MPU_MAG_STATUS1_REG        0x02
#define MPU_MAG_HXL_REG            0x03
#define MPU_MAG_HXH_REG            0x04
#define MPU_MAG_HYL_REG            0x05
#define MPU_MAG_HYH_REG            0x06
#define MPU_MAG_HZL_REG            0x07
#define MPU_MAG_HZH_REG            0x08
#define MPU_MAG_STATUS2_REG        0x09
#define MPU_MAG_CNTL1_REG          0x0A
#define MPU_MAG_CNTL2_REG          0x0B
#define MPU_MAG_ASTC_REG           0x0C
#define MPU_MAG_TS1_REG            0x0D
#define MPU_MAG_TS2_REG            0x0E
#define MPU_MAG_I2CDIS_REG         0x0F
#define MPU_MAG_ASAX_REG           0x10
#define MPU_MAG_ASAY_REG           0x11
#define MPU_MAG_ASAZ_REG           0x12

/* MPU Registers */
#define MPU_WHO_AM_I              0x71

#define ACCEL_XOUT_H              0x3B
#define ACCEL_XOUT_L              0x3C
#define ACCEL_YOUT_H              0x3D
#define ACCEL_YOUT_L              0x3E
#define ACCEL_ZOUT_H              0x3F
#define ACCEL_ZOUT_L              0x40

#define GYRO_XOUT_H               0x43
#define GYRO_XOUT_L               0x44
#define GYRO_YOUT_H               0x45
#define GYRO_YOUT_L               0x46
#define GYRO_ZOUT_H               0x47
#define GYRO_ZOUT_L               0x48

#define MPU_USER_CTRL             0x6A
#define MPU_PWR_MGMT_1            0x6B
#define MPU_PWR_MGMT_2            0x6C
#define MPU_XA_OFFSET_H           0x77
#define MPU_XA_OFFSET_L           0x78
#define MPU_YA_OFFSET_H           0x7A
#define MPU_YA_OFFSET_L           0x7B
#define MPU_ZA_OFFSET_H           0x7D
#define MPU_ZA_OFFSET_L           0x7E

#define CALIBRATION_SAMPLES      20

#define RAD_TO_DEG 57.2957786
Screen_HX8353E myScreen;

/* Kalman Filter Variables */
#define short

#ifdef KALMAN
Kalman kalmanX;
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle;
double compAngleX, compAngleY;
double kalAngleX, kalAngleY;
#endif

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;



unsigned char masterRxBuffer[SPI_MSG_LENGTH];
unsigned char masterTxBuffer[SPI_MSG_LENGTH];

extern "C" {
    /* Wrapper functions to call Clock::tick() */
    extern void *mainThread(void *arg0);
    void pull_data_push_visuals();
    void SPI_setup();
} // end extern "C"
/*
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
SPI_Handle      masterSpi;
SPI_Transaction transaction;

int32_t tmpxA = 0;
int32_t tmpyA = 0;
int32_t tmpzA = 0;
double presD;
double tempD;
double mpuData[6];
int32_t tmpxG = 0;
int32_t tmpyG = 0;
int32_t tmpzG = 0;
int16_t imuXcal[CALIBRATION_SAMPLES];
int16_t imuYcal[CALIBRATION_SAMPLES];
int16_t imuZcal[CALIBRATION_SAMPLES];

uint32_t timer;
uint32_t timer1;


bool            transferOK;

void reverse(char str[], int length)
{
    int start = 0;
    int end = length -1;
    while (start < end)
    {
        std::swap(*(str+start), *(str+end));
        start++;
        end--;
    }
}

// Implementation of itoa()
char* itoa(int num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;

    /* Handle 0 explicitely, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // In standard itoa(), negative numbers are handled only with
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10)
    {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }

    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';

    str[i] = '\0'; // Append string terminator

    // Reverse the string
    reverse(str, i);

    return str;
}

void SPI_setup()
{
    SPI_Params      spiParams;
    uint32_t        i;


    /* Open SPI as master (default) */
    init();
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA0;
    masterSpi = SPI_open(Board_SPI_MASTER, &spiParams);
    GPIO_write(Board_SPI_MPU_CS, 1);
    GPIO_write(Board_SPI_BMP_CS, 1);
    GPIO_write(Board_SPI_LCD_CS, 1);


    /*
     * Master has opened Board_SPI_MASTER; set Board_SPI_MASTER_READY high to
     * inform the slave.
     */
#if 0
    /* Read in the calibration data for BMP register dig_T1 both for LSB and MSB */
  /*  memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    masterTxBuffer[0] = BMP_DIG_T1_LSB_REG;
    transaction.count = 25;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(Board_SPI_BMP_CS, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(Board_SPI_BMP_CS, 1);*/
#endif
    spiWrite(BMP_DIG_T1_LSB_REG, 0, 25, Board_SPI_BMP_CS);
    dig_T1 = masterRxBuffer[1] | (masterRxBuffer[2] << 8);
    dig_T2 = masterRxBuffer[3] | (masterRxBuffer[4] << 8);
    dig_T3 = masterRxBuffer[5] | (masterRxBuffer[6] << 8);
    dig_P1 = masterRxBuffer[7] | (masterRxBuffer[8] << 8);
    dig_P2 = masterRxBuffer[9] | (masterRxBuffer[10] << 8);
    dig_P3 = masterRxBuffer[11] | (masterRxBuffer[12] << 8);
    dig_P4 = masterRxBuffer[13] | (masterRxBuffer[14] << 8);
    dig_P5 = masterRxBuffer[15] | (masterRxBuffer[16] << 8);
    dig_P6 = masterRxBuffer[17] | (masterRxBuffer[18] << 8);
    dig_P7 = masterRxBuffer[19] | (masterRxBuffer[20] << 8);
    dig_P8 = masterRxBuffer[21] | (masterRxBuffer[22] << 8);
    dig_P9 = masterRxBuffer[23] | (masterRxBuffer[24] << 8);
#if 0
    /* Reset MPU */
    /*memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    masterTxBuffer[0] = MPU_PWR_MGMT_1  & ~(0x80);
    masterTxBuffer[1] = 0x80;
    transaction.count = 2;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(Board_SPI_MPU_CS, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(Board_SPI_MPU_CS, 1);*/
#endif
    spiWrite(MPU_PWR_MGMT_1  & ~(0x80), 0x80, 2, Board_SPI_MPU_CS);

#if 0
    /* Change user control register */
    /*memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    masterTxBuffer[0] = MPU_USER_CTRL  & ~(0x80);
    masterTxBuffer[1] = 0b00010000;
    transaction.count = 2;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(Board_SPI_MPU_CS, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(Board_SPI_MPU_CS, 1);*/
#endif
    spiWrite(MPU_USER_CTRL  & ~(0x80), 0b00010000, 2, Board_SPI_MPU_CS);

    delay(500000);

    /* Find average accelerometer values at rest */

    for(i = 0; i < CALIBRATION_SAMPLES; i++)
    {
#if 0
        /* Read in accelerometer data */
       /* memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
        memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
        masterTxBuffer[0] = ACCEL_XOUT_H | 0x80;
        transaction.count = 7;
        transaction.txBuf = (void *) masterTxBuffer;
        transaction.rxBuf = (void *) masterRxBuffer;
        GPIO_write(Board_SPI_MPU_CS, 0);
        transferOK = SPI_transfer(masterSpi, &transaction);
        GPIO_write(Board_SPI_MPU_CS, 1);*/
#endif
        spiWrite(ACCEL_XOUT_H | 0x80, 0, 7, Board_SPI_MPU_CS);
        imuXcal[i] = (masterRxBuffer[1] << 8) | masterRxBuffer[2];
        imuYcal[i] = (masterRxBuffer[3] << 8) | masterRxBuffer[4];
        imuZcal[i] = (masterRxBuffer[5] << 8) | masterRxBuffer[6];
        tmpxA += imuXcal[i];
        tmpyA += imuYcal[i];
        tmpzA += imuZcal[i];
        delay(50000);
    }
/*
    for(i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        tmpxA += imuXcal[i];
        tmpyA += imuYcal[i];
        tmpzA += imuZcal[i];
    }*/
    tmpxA = (tmpxA/CALIBRATION_SAMPLES);
    tmpyA = (tmpyA/CALIBRATION_SAMPLES);
    tmpzA = (tmpzA/CALIBRATION_SAMPLES);
    mpuData[0] = tmpxA;
    mpuData[1] = tmpyA;
    mpuData[2] = tmpzA;



    /* Find average gyroscope values at rest */
    for(i = 0; i < CALIBRATION_SAMPLES; i++)
     {
#if 0
         /* Read in accelerometer data */
        /* memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
         memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
         masterTxBuffer[0] = GYRO_XOUT_H | 0x80;
         transaction.count = 7;
         transaction.txBuf = (void *) masterTxBuffer;
         transaction.rxBuf = (void *) masterRxBuffer;
         GPIO_write(Board_SPI_MPU_CS, 0);
         transferOK = SPI_transfer(masterSpi, &transaction);
         GPIO_write(Board_SPI_MPU_CS, 1);*/
#endif
         spiWrite(GYRO_XOUT_H | 0x80, 0, 7, Board_SPI_MPU_CS);
         imuXcal[i] = (masterRxBuffer[1] << 8) | masterRxBuffer[2];
         imuYcal[i] = (masterRxBuffer[3] << 8) | masterRxBuffer[4];
         imuZcal[i] = (masterRxBuffer[5] << 8) | masterRxBuffer[6];
         tmpxG += imuXcal[i];
         tmpyG += imuYcal[i];
         tmpzG += imuZcal[i];
         delay(50000);
     }

     /*for(i = 0; i < CALIBRATION_SAMPLES; i++)
     {
         tmpxG += imuXcal[i];
         tmpyG += imuYcal[i];
         tmpzG += imuZcal[i];
     }*/
     tmpxG = (tmpxG/CALIBRATION_SAMPLES);
     tmpyG = (tmpyG/CALIBRATION_SAMPLES);
     tmpzG = (tmpzG/CALIBRATION_SAMPLES);
     mpuData[3] = tmpxG;
     mpuData[4] = tmpyG;
     mpuData[5] = tmpzG;
#ifdef KALMAN
    setupFilter();
#endif
    myScreen.begin();
    myScreen.setOrientation(3);
    myScreen.rectangle(21, 64, 95, 22, whiteColour);
    myScreen.circle(80, 31, 7, violetColour);             // red = blue, green = green, yellow = cyan, cyan = yellow, teal = orange, magenta = purple, pink = violet,
    myScreen.circle(80, 31, 5, whiteColour);
    myScreen.circle(80, 31, 3, yellowColour);
    myScreen.circle(80, 31, 1, whiteColour);

}

void pull_data_push_visuals()
{
#ifndef short
    uint32_t data = 0;
    int32_t temp = 0;
    uint32_t pres = 0;
    int16_t accelX = 0;
    int16_t accelY = 0;
    int16_t accelZ = 0;

    /* Force a temperature measurement */
    masterTxBuffer[0] = BMP_CTRL_MEAS_REG & ~(0b10000000);
    masterTxBuffer[1] = 0b00100101;
    memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    transaction.count = 2;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(Board_SPI_BMP_CS, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(Board_SPI_BMP_CS, 1);



    /* Read in the temperature data */
    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    masterTxBuffer[0] = BMP_TEMP_MSB_REG;
    transaction.count = 4;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(Board_SPI_BMP_CS, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(Board_SPI_BMP_CS, 1);
    data = masterRxBuffer[3];
    data = data | (masterRxBuffer[1] << 12 | masterRxBuffer[2] << 4);
    temp = calculate_BMP_temp(data);
    tempD = (double)temp/100;



    /* Read in the pressure data */
    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    masterTxBuffer[0] = BMP_PRESS_MSB_REG;
    transaction.count = 4;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(Board_SPI_BMP_CS, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(Board_SPI_BMP_CS, 1);
    data = masterRxBuffer[3];
    data = data | (masterRxBuffer[1] << 12 | masterRxBuffer[2] << 4);
    pres = calculate_BMP_pres(data);
    presD = pres/256;



    /* Read in accelerometer data */
    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    masterTxBuffer[0] = ACCEL_XOUT_H | 0x80;
    transaction.count = 7;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(Board_SPI_MPU_CS, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(Board_SPI_MPU_CS, 1);
    accelX = (((masterRxBuffer[1] << 8) | masterRxBuffer[2]) - tmpxA);
    accelY = (((masterRxBuffer[3] << 8) | masterRxBuffer[4]) - tmpyA);
    accelZ = (((masterRxBuffer[5] << 8) | masterRxBuffer[6]) - tmpzA);
    mpuData[0] = (double)accelX / 16384;
    mpuData[1] = (double)accelY / 16384;
    mpuData[2] = (double)accelZ / 16384;


    /* Read in gyroscope data */
    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    masterTxBuffer[0] = GYRO_XOUT_H | 0x80;
    transaction.count = 7;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(Board_SPI_MPU_CS, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(Board_SPI_MPU_CS, 1);
    accelX = (((masterRxBuffer[1] << 8) | masterRxBuffer[2]) - tmpxG);
    accelY = (((masterRxBuffer[3] << 8) | masterRxBuffer[4]) - tmpyG);
    accelZ = (((masterRxBuffer[5] << 8) | masterRxBuffer[6]) - tmpzG);
    mpuData[3] = (double)accelX / 131;
    mpuData[4] = (double)accelY / 131;
    mpuData[5] = (double)accelZ / 131;

#endif
#ifdef short
    spiWrite(BMP_CTRL_MEAS_REG & ~(0b10000000), 0b00100101, 2, Board_SPI_BMP_CS);
    spiWrite(BMP_TEMP_MSB_REG, 0, 4, Board_SPI_BMP_CS);
    tempD = (double)(calculate_BMP_temp(masterRxBuffer[3] | (masterRxBuffer[1] << 12 | masterRxBuffer[2] << 4)))/100;

    spiWrite(BMP_PRESS_MSB_REG, 0, 4, Board_SPI_BMP_CS);
    presD = (double)(calculate_BMP_pres(masterRxBuffer[3] | (masterRxBuffer[1] << 12 | masterRxBuffer[2] << 4)))/256;

    spiWrite(ACCEL_XOUT_H | 0x80, 0, 7, Board_SPI_MPU_CS);
    /*
    mpuData[0] = (double)((((masterRxBuffer[1] << 8) | masterRxBuffer[2]) - tmpxA))/ 16384;
    mpuData[1] = (double)((((masterRxBuffer[3] << 8) | masterRxBuffer[4]) - tmpyA))/ 16384;
    mpuData[2] = (double)((((masterRxBuffer[5] << 8) | masterRxBuffer[6]) - tmpzA))/ 16384;*/
    int16_t accelX = 0;
    int16_t accelY = 0;
    int16_t accelZ = 0;
    accelX = (((masterRxBuffer[1] << 8) | masterRxBuffer[2]) - tmpxA);
    accelY = (((masterRxBuffer[3] << 8) | masterRxBuffer[4]) - tmpyA);
    accelZ = (((masterRxBuffer[5] << 8) | masterRxBuffer[6]) - tmpzA);
    mpuData[0] = (double)accelX / 16384;
    mpuData[1] = (double)accelY / 16384;
    mpuData[2] = (double)accelZ / 16384;

    spiWrite(GYRO_XOUT_H | 0x80, 0, 7, Board_SPI_MPU_CS);
    /*mpuData[3] = (double)((((masterRxBuffer[1] << 8) | masterRxBuffer[2]) - tmpxG))/ 131;
    mpuData[4] = (double)((((masterRxBuffer[3] << 8) | masterRxBuffer[4]) - tmpyG))/ 131;
    mpuData[5] = (double)((((masterRxBuffer[5] << 8) | masterRxBuffer[6]) - tmpzG))/ 131;*/
    accelX = (((masterRxBuffer[1] << 8) | masterRxBuffer[2]) - tmpxG);
    accelY = (((masterRxBuffer[3] << 8) | masterRxBuffer[4]) - tmpyG);
    accelZ = (((masterRxBuffer[5] << 8) | masterRxBuffer[6]) - tmpzG);
    mpuData[3] = (double)accelX / 131;
    mpuData[4] = (double)accelY / 131;
    mpuData[5] = (double)accelZ / 131;
#endif
#ifdef KALMAN
    filterData();
#endif
    uint32_t period, time;
    period = ti_sysbios_knl_Clock_getTickPeriod__E();
    time = ti_sysbios_knl_Clock_getTicks__E();
    double dt = ( (double)(time*10) - (double)(timer*10) )/1000000;
    timer = time;
    double y = abs((double)((double)mpuData[1]*9.81)*dt);//((double)((time*10) - (timer*10))/1000000);
    uint32_t yINT = y;
    double yDEC = (y - yINT)*100;
    /* Display data to LCD and to UART terminal */
    char bufferOut[10];
    myScreen.gText(22, 24, (String)itoa(tempD, bufferOut, 10) + " °C", whiteColour);
    myScreen.gText(22, 32, (String)itoa(presD, bufferOut, 10) + " Pa", whiteColour);
    myScreen.gText(22, 40, (String)itoa(yINT, bufferOut, 10) + "." + (String)itoa(yDEC, bufferOut, 10) +  "m/s  ", whiteColour);
    myScreen.gText(22, 48, "Pitch:"  + (String)itoa(kalAngleX, bufferOut, 10) + "°  ", whiteColour);
    myScreen.gText(22, 56, "Roll:"  + (String)itoa(kalAngleY, bufferOut, 10) + "°  ", whiteColour);
   // myScreen.gText(2, 54, "YA: "  + (String)itoa(mpuData[1]*9.81, bufferOut, 10) + "m/s/s     ", whiteColour);
    //myScreen.gText(2, 62, "ZA: "  + (String)itoa(mpuData[2]*9.81, bufferOut, 10) + "m/s/s     ", whiteColour);
    //myScreen.gText(2, 38, "X: "        + (String)itoa(kalAngleX, bufferOut, 10) + " °", whiteColour);
   // myScreen.gText(2, 46, "Y: "        + (String)itoa(kalAngleY, bufferOut, 10) + " °", whiteColour);


   //myScreen.gText(2, 70, "Ac(mg):"    + (String)itoa(mpuData[0]*1000, bufferOut, 10) + (String)itoa(mpuData[1]*1000, bufferOut, 10) + (String)itoa(mpuData[2]*1000, bufferOut, 10), whiteColour);
 //  myScreen.gText(2, 80, "Gy(md/s):" + (String)itoa(mpuData[3]*1000, bufferOut, 10) + (String)itoa(mpuData[4]*1000, bufferOut, 10) + (String)itoa(mpuData[5]*1000, bufferOut, 10), whiteColour);
    //Display_printf(display, 0, 0, "Pressure: %f Pa Temperature: %f %cC Accel X Y Z: %f %f %f g Gyro X Y Z: %f %f %f %c/s", presD, tempD, 248, mpuData[0], mpuData[1], mpuData[2], mpuData[3], mpuData[4], mpuData[5], 248);

}
void spiWrite(uint16_t reg, uint16_t val, uint8_t cnt, uint8_t cs)
{
    masterTxBuffer[0] = reg;
    masterTxBuffer[1] = val;
    memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    transaction.count = cnt;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    GPIO_write(cs, 0);
    transferOK = SPI_transfer(masterSpi, &transaction);
    GPIO_write(cs, 1);
    return;
}

#ifdef KALMAN
void filterData()
{

    uint32_t time;
    time = ti_sysbios_knl_Clock_getTicks__E();
    double dt = ( (double)(time*10) - (double)(timer*10) )/1000000;
    timer1 = time;
    /*
    uint32_t period, time;
    period = ti_sysbios_knl_Clock_getTickPeriod__E();
    time = ti_sysbios_knl_Clock_getTicks__E();

    double dt = ((time*period) - (timer*period))/1000000;*/

    double roll = atan2(mpuData[1], mpuData[0]) * RAD_TO_DEG;
    double pitch = atan(-mpuData[2]/sqrt(mpuData[1] * mpuData[1] + mpuData[0] * mpuData[0])) * RAD_TO_DEG;

    double gyroXrate = mpuData[3] / 131;
    double gyroYrate = mpuData[4] / 131;

    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    }
    else
    {
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }
    if (abs(kalAngleX) > 90)
    {
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    gyroXangle += gyroXrate * dt;
    gyroYangle += gyroYrate * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
}

void setupFilter()
{
    double roll  = atan2(mpuData[1], mpuData[0]) * RAD_TO_DEG;
    double pitch = atan(-mpuData[2] / sqrt(mpuData[1] * mpuData[1] + mpuData[0] * mpuData[0])) * RAD_TO_DEG;

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer1 = ti_sysbios_knl_Clock_getTicks__E();
}
#endif




int32_t t_fine;
int32_t calculate_BMP_temp(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t calculate_BMP_pres(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19; p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
    return (uint32_t)p;
}

uint8_t spi_send(uint8_t data)
{
    memset((void *) masterTxBuffer, 0, SPI_MSG_LENGTH);
    memset((void *) masterRxBuffer, 0, SPI_MSG_LENGTH);
    masterTxBuffer[0] = data;
    transaction.count = 1;
    transaction.txBuf = (void *) masterTxBuffer;
    transaction.rxBuf = (void *) masterRxBuffer;
    return SPI_transfer(masterSpi, &transaction);
}

/*
 * Extern "C" block to prevent name mangling
 * of functions called within the Configuration Tool
 */
void init()
{




    /* Call driver init functions. */
    Display_init();
    GPIO_init();
    SPI_init();

    /* Configure the LED pins */
    GPIO_setConfig(Board_SPI_BMP_CS, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_SPI_LCD_CS, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_SPI_MPU_CS, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Open the display for output */
   // display = Display_open(Display_Type_UART, NULL);
#if 0
    if (display == NULL) {
        /* Failed to open display driver */
        while (1);
    }


    Display_printf(display, 0, 0, "Starting the SPI master example");
    Display_printf(display, 0, 0, "This example requires external wires to be "
        "connected to the header pins. Please see the Board.html for details.\n");

    /* Create application threads */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create master thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, masterThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
    #endif

    return;
}





