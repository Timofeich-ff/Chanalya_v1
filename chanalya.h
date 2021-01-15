/* --------------------------------------------------------------------------------------
File name:	chanalya.h
Author:		Maksimov Timofey
Desc:		Library pack for Chanalya
Date:		09.01.2021
Rel:		1.0
-------------------------------------------------------------------------------------- */

#ifndef __CHANALYA__
#define __CHANALYA__

#define TICK_FLAG_DELTA 1000
//#define TICK_FLAG_DELTA_DIV 10

#define NONE 0
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 2

#define ADC_COEF 883 // adjust here ADC coefficient

typedef struct {
	int pin;
	void * port;
} hw_dio_t;

typedef struct {
	hw_dio_t cs1;
	hw_dio_t cs2;
	hw_dio_t cs3;
	hw_dio_t cs4;
	hw_dio_t cs5;
	hw_dio_t cs6;
	hw_dio_t cs7;
	hw_dio_t cs8;
	hw_dio_t cs9;
	hw_dio_t cs10;
	hw_dio_t cs11;
	hw_dio_t cs12;
	
	hw_dio_t reset;	 
	hw_dio_t rst_out; 
	void * spi;	
} hw_594_t;

void Buzz(int time);
void Reg594_sendByte(hw_594_t * hw, uint8_t byte);
void Reg594_set(hw_594_t * hw);
void Reg594_reset(hw_594_t * hw);

#endif /* __CHANALYA__ */
