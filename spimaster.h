/*
 * spimaster.h
 *
 *  Created on: Mar 14, 2019
 *      Author: zacas
 */

#ifndef SPIMASTER_H_
#define SPIMASTER_H_

int32_t calculate_BMP_temp(int32_t adc_T);
uint32_t calculate_BMP_pres(int32_t adc_P);
uint8_t spi_send(uint8_t data);
void init();
void setupFilter();
void filterData();
void spiWrite(uint16_t reg, uint16_t val, uint8_t cnt, uint8_t cs);


//void pull_data_push_visuals();

#endif /* SPIMASTER_H_ */
