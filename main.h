/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SW_Pin GPIO_PIN_13
#define SW_GPIO_Port GPIOC
#define DT_Pin GPIO_PIN_14
#define DT_GPIO_Port GPIOC
#define CLK_Pin GPIO_PIN_15
#define CLK_GPIO_Port GPIOC
#define BAT_CHG_Pin GPIO_PIN_0
#define BAT_CHG_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_1
#define CS1_GPIO_Port GPIOA
#define CS9_Pin GPIO_PIN_2
#define CS9_GPIO_Port GPIOA
#define CS5_Pin GPIO_PIN_3
#define CS5_GPIO_Port GPIOA
#define RESET_Pin GPIO_PIN_4
#define RESET_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define RST_OUT_Pin GPIO_PIN_6
#define RST_OUT_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define BUT3_Pin GPIO_PIN_0
#define BUT3_GPIO_Port GPIOB
#define BUT4_Pin GPIO_PIN_1
#define BUT4_GPIO_Port GPIOB
#define BUT1_Pin GPIO_PIN_10
#define BUT1_GPIO_Port GPIOB
#define BUT2_Pin GPIO_PIN_11
#define BUT2_GPIO_Port GPIOB
#define CS8_Pin GPIO_PIN_12
#define CS8_GPIO_Port GPIOB
#define CS12_Pin GPIO_PIN_13
#define CS12_GPIO_Port GPIOB
#define CS4_Pin GPIO_PIN_14
#define CS4_GPIO_Port GPIOB
#define CS7_Pin GPIO_PIN_15
#define CS7_GPIO_Port GPIOB
#define CS11_Pin GPIO_PIN_8
#define CS11_GPIO_Port GPIOA
#define CS3_Pin GPIO_PIN_9
#define CS3_GPIO_Port GPIOA
#define CS6_Pin GPIO_PIN_10
#define CS6_GPIO_Port GPIOA
#define CS10_Pin GPIO_PIN_11
#define CS10_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_12
#define CS2_GPIO_Port GPIOA
#define LCD_LED_Pin GPIO_PIN_5
#define LCD_LED_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_8
#define BUZZ_GPIO_Port GPIOB
#define BRA_IN_Pin GPIO_PIN_9
#define BRA_IN_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
