
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "chanalya.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//Chanalya: defines BEGIN
#define LED_on HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_RESET)
#define LED_off HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_SET)
//Chanalya: defines END

//Chanalya: variables BEGIN
unsigned int led = 1;  			// 0/1 - off/on
unsigned int mode = 0; 			// 0/1 - SCH(search)/DEF(define)
unsigned int l_ch_thd = 1;	  // 1-96 - lower chanel threshold
unsigned int u_ch_thd = 96; 	// 1-96 - upper chanel threshold
extern char symbol_0[];
extern char symbol_1[];
extern char symbol_2[];

//timer variables
unsigned long tick = 0; 																// current SysTick value
unsigned const long tick_flag_delta = TICK_FLAG_DELTA;	// timer delta in [msec]

//ADC variables
unsigned int adc = 0;
unsigned int adc_flag = 0;
unsigned long adc_tick_flag = 0; 	// ADC timer flag

//Chanalya: variables END
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	//Chanalya: ADC initialization BEGIN
	HAL_ADCEx_Calibration_Start(&hadc1);
	//Chanalya: ADC initialization END
	
	//Chanalya: shift registers initialization BEGIN
	hw_594_t hw_594;
	
	hw_594.cs1.pin = CS1_Pin;
	hw_594.cs2.pin = CS2_Pin;
	hw_594.cs3.pin = CS3_Pin;
	hw_594.cs4.pin = CS4_Pin;
	hw_594.cs5.pin = CS5_Pin;
	hw_594.cs6.pin = CS6_Pin;
	hw_594.cs7.pin = CS7_Pin;
	hw_594.cs8.pin = CS8_Pin;
	hw_594.cs9.pin = CS9_Pin;
	hw_594.cs10.pin = CS10_Pin;
	hw_594.cs11.pin = CS11_Pin;
	hw_594.cs12.pin = CS12_Pin;
	hw_594.cs1.port = CS1_GPIO_Port;
	hw_594.cs2.port = CS2_GPIO_Port;
	hw_594.cs3.port = CS3_GPIO_Port;
	hw_594.cs4.port = CS4_GPIO_Port;
	hw_594.cs5.port = CS5_GPIO_Port;
	hw_594.cs6.port = CS6_GPIO_Port;
	hw_594.cs7.port = CS7_GPIO_Port;
	hw_594.cs8.port = CS8_GPIO_Port;
	hw_594.cs9.port = CS9_GPIO_Port;
	hw_594.cs10.port = CS10_GPIO_Port;
	hw_594.cs11.port = CS11_GPIO_Port;
	hw_594.cs12.port = CS12_GPIO_Port;
	
	hw_594.reset.pin = RESET_Pin;
	hw_594.reset.port = RESET_GPIO_Port;
	hw_594.rst_out.pin = RST_OUT_Pin;
	hw_594.rst_out.port = RST_OUT_GPIO_Port;
	hw_594.spi = &hspi1;
	
	//Chanalya: shift registers initialization END
	
	//Chanalya: LCD initialization BEGIN
	lcd_init();
	lcd_clear();
	lcd_ostec();
	Buzz(100);
	HAL_Delay(1000);
	lcd_clear();
	HAL_Delay(100);
	lcd_menu();
	lcd_mode(mode);
	lcd_l_ch_thd(l_ch_thd);
	lcd_u_ch_thd(u_ch_thd);
	
	//lcd_prog_symbol (0x00, symbol_0);
	//lcd_put_cur(1, 3);
	//lcd_send_data (0x00); // symbol #0
	
	Buzz(100);
	//Chanalya: LCD initialization END
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
	// Chanalya: Timer 
	tick = HAL_GetTick(); 
	if (adc_flag == 1){
		if ((tick - adc_tick_flag) > tick_flag_delta)
		{
			adc_flag = 0;
		} else if ((tick - adc_tick_flag) == 0)
		{
			adc_tick_flag = tick;
		}
	}
	
	// Chanalya: ADC measurement, LCD reaction
	if (adc_flag == 0){
		HAL_ADC_Start(&hadc1); 
    HAL_ADC_PollForConversion(&hadc1, 100); 
    adc = HAL_ADC_GetValue(&hadc1); 
    HAL_ADC_Stop(&hadc1);
		
		unsigned int volt = (adc * 100) / ADC_COEF;
		lcd_adc (volt);
		
		adc_tick_flag = tick;
		adc_flag = 1;
	}
		
	// Chanalya: BUT1 (backlight) check, LCD reaction
	if(HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin) == GPIO_PIN_RESET){
		if(led == 0){
			LED_on;
			Buzz(20);	
			HAL_Delay(200);
			led=1;		
		} else {
			LED_off;
			Buzz(20);	
			HAL_Delay(200);
			led=0;	
		}
	}
	
	// Chanalya: BUT2 (mode) check, LCD reaction
	if(HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin) == GPIO_PIN_RESET){
		if(mode == 0){
			mode=1;
			lcd_mode(mode);
			Buzz(20);	
			HAL_Delay(200);
		} else {			
			mode=0;
			lcd_mode(mode);
			Buzz(20);	
			HAL_Delay(200);
		}
	}
	
	// Chanalya: BUT3 (lower threshold) check, LCD reaction 
	if(HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin) == GPIO_PIN_RESET){
		// erase of 'L' sign 
		lcd_put_cur(1, 9);
		lcd_send_data (0x20); // ''
		Buzz(20);	
		HAL_Delay(200);
		
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
		int prev_counter, next_counter = TIM2->CNT;
		l_ch_thd > 96 ? l_ch_thd = 96 : l_ch_thd++;
		
		// encoder managment
		while (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin) == GPIO_PIN_SET &&
					 HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET){
			next_counter = TIM2->CNT;			 
			if(next_counter - prev_counter >= 1){
				prev_counter=next_counter;
				l_ch_thd >= u_ch_thd ? l_ch_thd = u_ch_thd : l_ch_thd++;				
				lcd_l_ch_thd (l_ch_thd);
			} else if (prev_counter - next_counter >= 1){
				prev_counter=next_counter;
				l_ch_thd == 0 ? l_ch_thd = 1 : l_ch_thd--;
				lcd_l_ch_thd (l_ch_thd);
			}	
		}
		
		HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
		// retrieve of 'L' sign 
		lcd_put_cur(1, 9);
		lcd_send_data (0x4C); // 'L'
		Buzz(20);	
		HAL_Delay(200);
	}
	
	// Chanalya: BUT4 (upper threshold) check, LCD reaction
	if(HAL_GPIO_ReadPin(BUT4_GPIO_Port, BUT4_Pin) == GPIO_PIN_RESET){
		// erase of 'U' sign 
		lcd_put_cur(1, 13);
		lcd_send_data (0x20); // ''
		Buzz(20);	
		HAL_Delay(200);
		
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
		int prev_counter, next_counter = TIM2->CNT;
		u_ch_thd > 96 ? u_ch_thd = 96 : u_ch_thd++;
		
		// encoder managment 
		while (HAL_GPIO_ReadPin(BUT4_GPIO_Port, BUT4_Pin) == GPIO_PIN_SET &&
					 HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET){
			next_counter = TIM2->CNT;			 
			if(next_counter - prev_counter >= 1){
				prev_counter=next_counter;
				u_ch_thd > 96 ? u_ch_thd = 96 : u_ch_thd++;				
				lcd_u_ch_thd (u_ch_thd);
			} else if (prev_counter - next_counter >= 1){
				prev_counter=next_counter;
				u_ch_thd <= l_ch_thd ? u_ch_thd = l_ch_thd : u_ch_thd--;
				lcd_u_ch_thd (u_ch_thd);
			}	
		}	
					 
		HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
		// retrieve of 'U' sign 
		lcd_put_cur(1, 13);
		lcd_send_data (0x55); // 'U'
		Buzz(20);	
		HAL_Delay(200);
	}
	
	// Chanalya: encoder button, LCD reaction
	if(HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET){
		lcd_put_cur(0, 7);
		lcd_send_data (0x7F); // '<-'
		Buzz(20);	
		HAL_Delay(200);
		
		if(mode == 0){
			// Start DEF (define) mode
			while (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET){
				int i = (l_ch_thd-1);
				while(i < u_ch_thd){
					Reg594_sendByte(&hw_594,i);
					HAL_Delay(1);
					if(HAL_GPIO_ReadPin(BRA_IN_GPIO_Port, BRA_IN_Pin) == GPIO_PIN_RESET){
						lcd_channel(i+1);
						Buzz(200);
						lcd_channel(0);
					}
					Reg594_reset(&hw_594);
					if(HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET){break;}
					i++;
				}
			}
			Reg594_reset(&hw_594);
			lcd_put_cur(0, 7);
			lcd_send_data (0x20); // ''
			Buzz(20);	
			HAL_Delay(200);
		} else {
			// displaying of '1' channel 
			int channel = l_ch_thd+1;
			lcd_put_cur(0, 6);
			lcd_send_data (0x7F); // '<-'
			lcd_channel(channel);
		
			HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
			int prev_counter, next_counter = TIM2->CNT;
			
			// encoder managment
			while (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET){
				next_counter = TIM2->CNT;			 
				if(next_counter - prev_counter >= 1){
					prev_counter=next_counter;	
					channel >= u_ch_thd ? channel = u_ch_thd : channel++;					
					lcd_channel(channel);
				} else if (prev_counter - next_counter >= 1){
					prev_counter=next_counter;
					channel <= l_ch_thd ? channel = l_ch_thd : channel--;
					lcd_channel(channel);
				}	
			}
			
			//stop encoder managment
			HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
			// retrieve of 'U' sign 
			lcd_put_cur(0, 6);
			lcd_send_data (' '); 
			Buzz(20);	
			HAL_Delay(200);
			
			// Start SCH (search) mode
			while (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET){
				for(int i=(l_ch_thd-1); i<u_ch_thd; i++){
					Reg594_sendByte(&hw_594,i);
					HAL_Delay(1);
					if(HAL_GPIO_ReadPin(BRA_IN_GPIO_Port, BRA_IN_Pin) == GPIO_PIN_RESET){
						if((i+1) == channel){
							lcd_put_cur(0, 5);
							lcd_send_data (0x3C); // '<'
							lcd_put_cur(0, 6);
							lcd_send_data (0x3C); // '<'
							Buzz(200);
							lcd_put_cur(0, 5);
							lcd_send_data (0x20); // ' '
							lcd_put_cur(0, 6);
							lcd_send_data (0x20); // ' '
						}
					}
					Reg594_reset(&hw_594);
					if(HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET){break;}
				}
			}
			Reg594_reset(&hw_594);
			lcd_channel(0);
			lcd_put_cur(0, 7);
			lcd_send_data (0x20); // ''
			Buzz(20);	
			HAL_Delay(200);
		}
	}
	
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
