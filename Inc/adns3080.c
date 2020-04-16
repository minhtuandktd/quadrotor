#include "adns3080.h"
#include "dwt_stm32_delay.h"

#define ADNS_ENABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
#define ADNS_DISABLE	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

void adns_init(SPI_HandleTypeDef hspi){
	uint8_t product_id_reg = 0x00, pid;
	
	ADNS_DISABLE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	DWT_Delay_ms(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	DWT_Delay_ms(50);

	ADNS_ENABLE
	DWT_Delay_us(2);
	HAL_SPI_Transmit(&hspi, &product_id_reg, 1, 1);
	DWT_Delay_us(1);
	HAL_SPI_Receive(&hspi, &pid, 1, 1);
	DWT_Delay_us(1);
	if (pid == 0x17){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	}
	ADNS_DISABLE
	DWT_Delay_ms(1);
	
	ADNS_ENABLE
	DWT_Delay_ms(1);
	uint8_t configuration_bits = 0x8a, configuration_value = 0x19;
	HAL_SPI_Transmit(&hspi, &configuration_bits, 1, 1);
	DWT_Delay_us(1);
	HAL_SPI_Transmit(&hspi, &configuration_value, 1, 1);
	DWT_Delay_us(1);
	ADNS_DISABLE
	DWT_Delay_ms(1);
	
	// ADNS_ENABLE
	// uint8_t configuration_bits_read = 0x0a;
	// HAL_SPI_Transmit(&hspi1, &configuration_bits_read, 1, 1);
	// DWT_Delay_us(1);
	// HAL_SPI_Receive(&hspi1, &test_configuration, 1, 1);
	// DWT_Delay_us(1);
	// ADNS_DISABLE
	// DWT_Delay_ms(1);
}
int8_t dx, dy;
int32_t position_x, position_y;

void adns_get_data(SPI_HandleTypeDef hspi){
	ADNS_ENABLE
//	DWT_Delay_us(10);
	//	delay_ms(1);
	uint8_t motion_reg = 0x02, motion;
	DWT_Delay_us(1);
	HAL_SPI_Transmit(&hspi, &motion_reg, 1, 1);
	DWT_Delay_us(1);
	HAL_SPI_Receive(&hspi, &motion, 1, 1);
	DWT_Delay_us(1);
	ADNS_DISABLE
	
	if (motion >> 7){
		uint8_t dx_reg = 0x03, dx_raw;
		DWT_Delay_us(1);
		ADNS_ENABLE
		DWT_Delay_us(1);
		HAL_SPI_Transmit(&hspi, &dx_reg, 1, 1);
		DWT_Delay_us(1);
		HAL_SPI_Receive(&hspi, &dx_raw, 1, 1);
		DWT_Delay_us(1);
		ADNS_DISABLE
	
		uint8_t dy_reg = 0x04, dy_raw;
		ADNS_ENABLE
		DWT_Delay_us(1);
		HAL_SPI_Transmit(&hspi, &dy_reg, 1, 1);
		DWT_Delay_us(1);
		HAL_SPI_Receive(&hspi, &dy_raw, 1, 1);
		DWT_Delay_us(1);
		ADNS_DISABLE
		DWT_Delay_us(1);
		
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	
		if (dx_raw >= 128) dx = dx_raw - 256;
		else dx = dx_raw;
		if (dy_raw >= 128) dy = dy_raw - 256;
		else dy = dy_raw;
	}
	else dx = dy = 0;
	DWT_Delay_us(1);

}

int32_t get_position_x(void){
	return position_x += dx;
}

int32_t get_position_y(void){
	return position_y += dy;
}
