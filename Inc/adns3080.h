#ifndef __ADNS3080_H__
#define __ADNS3080_H__

#include "stm32f4xx_hal.h"
#include "main.h"

void adns_init(SPI_HandleTypeDef hspi);
void adns_get_data(SPI_HandleTypeDef hspi);
int32_t get_position_x(void);
int32_t get_position_y(void);

#endif
