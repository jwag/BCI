/**
  ******************************************************************************
  * @file    SysTick/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// ADC Variables
__IO uint16_t ADC3ConvertedValues[NUM_SENSORS*BUFFER_SIZE] = {};
__IO uint16_t avg_value[NUM_SENSORS] = {};
__IO uint16_t avg_voltage[NUM_SENSORS] = {};
GPIO_InitTypeDef GPIO_InitStructure;
static __IO uint32_t TimingDelay;
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

char volatile start_packet[] = "<({";

// PWM Setup Constants
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t CCR1_Val = 333;
uint16_t CCR2_Val = 249;
uint16_t CCR3_Val = 166;
uint16_t CCR4_Val = 83;
uint16_t PrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);
void TIM_Config(void);
void ADC3_CH12_DMA_Config(void);
void PWM_Config();

/* Private functions ---------------------------------------------------------*/

//***************************************************************************************
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  /* Initialize Leds mounted on STM32F4-Discovery board */
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  init_USART1(9600); // initialize USART1 @ 9600 baud

  char hello[]  = "Init complete! Hello World!/n";
  USART_puts_chars(USART1, hello); // just send a message to indicate that it works


  /* ADC3 configuration *******************************************************/
  /*  - Enable peripheral clocks                                              */
  /*  - DMA2_Stream0 channel2 configuration                                   */
  /*  - Configure ADC Channel12 pin as analog input                           */
  /*  - Configure ADC3 Channel12                                              */
  ADC3_CH12_DMA_Config();

  /* Start ADC3 Software Conversion */
  ADC_SoftwareStartConv(ADC3);

  DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);

  /* TIM Configuration */
  TIM_Config();

  PWM_Config();

  /* Setup SysTick Timer for 1 msec interrupts.
     ------------------------------------------
    1. The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the lowest value (0x0F).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.

    2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
       SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
       SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
       inside the misc.c file.

    3. You can change the SysTick IRQ priority by calling the
       NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function
       call. The NVIC_SetPriority() is defined inside the core_cm4.h file.

    4. To adjust the SysTick time base, use the following formula:

         Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)

       - Reload Value is the parameter to be passed for SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF
   */
  if (SysTick_Config(SystemCoreClock / (6250)))
  {
    /* Capture error */
    while (1);
  }

  static uint8_t duty_cycle = 0;
  while (1)
  {
    /* Toggle LED3 and LED6 */
    STM_EVAL_LEDToggle(LED3);
    Delay(100);
    STM_EVAL_LEDToggle(LED4);
    Delay(100);
    STM_EVAL_LEDToggle(LED6);
    Delay(100);
    STM_EVAL_LEDToggle(LED5);
    Delay(100);
  }
}
//***************************************************************************************
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}
//***************************************************************************************
/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
//***************************************************************************************

/**
  * @brief  ADC3 channel12 with DMA configuration
  * @param  None
  * @retval None
  */
void ADC3_CH12_DMA_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, ADC1 DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_DeInit(DMA2_Stream0);
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValues[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = NUM_SENSORS*BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel12 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 2;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel12 configuration *************************************/
  // Pin PC2
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_480Cycles);
  // Pin PC3
  ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 2, ADC_SampleTime_480Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  //Enable DMA2 channel IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}
//***************************************************************************************
/**
  * @brief  Configure the TIM3 and TIM4 Ouput Channels.
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* TIM3 and TIM4 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    /* GPIOC and GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);

    /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) and TIM4 CH3 (PB8) and TIM4 CH4 (PB9) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM3 pins to AF2 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

}
//***************************************************************************************
void PWM_Config()
{
    /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.

    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
    since APB1 prescaler is different from 1.
      TIM3CLK = 2 * PCLK1
      PCLK1 = HCLK / 4
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2

    To get TIM3 counter clock at 28 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /28 MHz) - 1

    To get TIM3 output clock at 30 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM3 counter clock / TIM3 output clock) - 1
           = 665

    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.
    ----------------------------------------------------------------------- */

    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 8900;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    // To set Duty cycle just set this and if 50% duty cycle is desired
    // Then take 8900*(decimal duty Cycle to get TIM_Pulse
    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 4450;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    //TIM4 Config
    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 2000;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init(TIM4, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 4450;

    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);
    /* TIM enable counter */
    TIM_Cmd(TIM4, ENABLE);
}
//*********************************************************************************
void set_PWM_duty( uint8_t duty_cycle,led_t led)
{

    uint16_t ccr = floor(duty_cycle*89.f);
    switch (led)
    {
        case LEFT_LED_880:
          {
              // PC6 LED
              TIM3->CCR1 = ccr;
              break;
          }
          case LEFT_LED_940:
          {
              // PC7 LED
              TIM3->CCR2 = ccr;
              break;
          }
          case CENTER_LED_880:
          {
              // PB0 LED
              TIM3->CCR3 = ccr;
              break;
          }
          case CENTER_LED_940:
          {
              // PB0 LED
              TIM3->CCR4 = ccr;
              break;
          }
          case RIGHT_LED_880:
          {
              // PB8 LED
              TIM4->CCR3 = ccr;
              break;
          }
          case RIGHT_LED_940:
          {
              // PB9 LED
              TIM4->CCR4 = ccr;
              break;
          }
          default:
          {
              break;
          }
    }

}

//*******************************************************************************
/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */
void init_USART1(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts_chars(USART_TypeDef* USARTx, volatile char *s)
{
	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}


// Sends 8 bit integers over serial.  Note one may send a float, but it must
// be cast and a uint8_t and reproceessed on the other end.
void USART_puts_ints(USART_TypeDef* USARTx, uint8_t *data, uint8_t length)
{
	for(int8_t i = 0;i <length;i++)
    {
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, data[i]);
	}
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
