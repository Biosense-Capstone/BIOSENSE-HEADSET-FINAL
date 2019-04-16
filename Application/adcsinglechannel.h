/*
 * adcsinglechannel.h
 *
 *  Created on: Apr 9, 2019
 *      Author: wulff
 */

#ifndef ADCSINGLECHANNEL_H_
#define ADCSINGLECHANNEL_H_
#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>
#include <stdint.h>
void BHB_adc_init();
void BHB_adc_close();
uint16_t BHB_adc_getVal();



#endif /* ADCSINGLECHANNEL_H_ */
