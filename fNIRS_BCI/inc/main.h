/**
  ******************************************************************************
  * @file    SysTick/main.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_usart.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    LEFT_LED_880,
    LEFT_LED_940,
    CENTER_LED_880,
    CENTER_LED_940,
    RIGHT_LED_880,
    RIGHT_LED_940
} led_t;

/* Exported constants --------------------------------------------------------*/
#define NUM_SENSORS 2
#define BUFFER_SIZE 100
#define MAX_STRLEN 12 // this is the maximum string length of our string in characters
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Exported variables ------------------------------------------------------- */
extern __IO uint16_t ADC3ConvertedValues[];
extern __IO uint16_t avg_value[NUM_SENSORS];
extern __IO uint16_t avg_voltage[NUM_SENSORS];
void TimingDelay_Decrement(void);
void init_USART1(uint32_t baudrate);
void USART_puts_chars(USART_TypeDef* USARTx, volatile char *s);
void USART_puts_ints(USART_TypeDef* USARTx, uint8_t *data, uint8_t length);
void set_PWM_duty( uint8_t duty_cycle,led_t led);
extern volatile char received_string[];

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
