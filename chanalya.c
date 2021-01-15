/* --------------------------------------------------------------------------------------
File name:	chanalya.c
Author:		Maksimov Timofey
Desc:		Library pack for Chanalya
Date:		09.01.2021
Rel:		1.0
-------------------------------------------------------------------------------------- */

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h" // !change for other STM32 family if necessary!

#include "chanalya.h"

/* Functions ------------------------------------------------------------------*/

// BUZZ() activates BUZZ for 20ms
void Buzz(int time){
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
	HAL_Delay(time);
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
}

// 74AHC594 register 
void Reg594_sendByte(hw_594_t * hw, uint8_t q){
	
	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(hw->rst_out.port, hw->rst_out.pin, GPIO_PIN_SET);
	
	uint8_t byte[1] = {0x01,};
	byte[0] <<= q%8; // q = 0-95
	
	HAL_SPI_Transmit(hw->spi, byte, 1, 100);
	while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY);
	
	uint8_t cs = q/8;
	if (cs == 0){
		HAL_GPIO_WritePin(hw->cs1.port, hw->cs1.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs1.port, hw->cs1.pin, GPIO_PIN_RESET);
	} else if (cs == 1){
		HAL_GPIO_WritePin(hw->cs2.port, hw->cs2.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs2.port, hw->cs2.pin, GPIO_PIN_RESET);
	} else if (cs == 2){
		HAL_GPIO_WritePin(hw->cs3.port, hw->cs3.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs3.port, hw->cs3.pin, GPIO_PIN_RESET);
	} else if (cs == 3){
		HAL_GPIO_WritePin(hw->cs4.port, hw->cs4.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs4.port, hw->cs4.pin, GPIO_PIN_RESET);
	} else if (cs == 4){
		HAL_GPIO_WritePin(hw->cs5.port, hw->cs5.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs5.port, hw->cs5.pin, GPIO_PIN_RESET);
	} else if (cs == 5){
		HAL_GPIO_WritePin(hw->cs6.port, hw->cs6.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs6.port, hw->cs6.pin, GPIO_PIN_RESET);
	} else if (cs == 6){
		HAL_GPIO_WritePin(hw->cs7.port, hw->cs7.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs7.port, hw->cs7.pin, GPIO_PIN_RESET);
	} else if (cs == 7){
		HAL_GPIO_WritePin(hw->cs8.port, hw->cs8.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs8.port, hw->cs8.pin, GPIO_PIN_RESET);
	} else if (cs == 8){
		HAL_GPIO_WritePin(hw->cs9.port, hw->cs9.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs9.port, hw->cs9.pin, GPIO_PIN_RESET);
	} else if (cs == 9){
		HAL_GPIO_WritePin(hw->cs10.port, hw->cs10.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs10.port, hw->cs10.pin, GPIO_PIN_RESET);
	} else if (cs == 10){
		HAL_GPIO_WritePin(hw->cs11.port, hw->cs11.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs11.port, hw->cs11.pin, GPIO_PIN_RESET);
	} else if (cs == 11){
		HAL_GPIO_WritePin(hw->cs12.port, hw->cs12.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(hw->cs12.port, hw->cs12.pin, GPIO_PIN_RESET);
	} 
}

void Reg594_set(hw_594_t * hw){
	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(hw->rst_out.port, hw->rst_out.pin, GPIO_PIN_SET);
}

void Reg594_reset(hw_594_t * hw){
	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(hw->rst_out.port, hw->rst_out.pin, GPIO_PIN_RESET);
}

