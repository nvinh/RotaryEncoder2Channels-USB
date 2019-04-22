/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Vinh Nguyen, Robotic Wheelchair project
	* Rotary Encoder 2 channels
  * All rights reserved.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

/* USER CODE BEGIN Includes */
TIM_HandleTypeDef htimLeft;
TIM_HandleTypeDef htimRight2;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define Left_Count                  ((int32_t)(0xFFFF))
__IO int32_t leftCounter = 0;
__IO int32_t leftValue = 0;
__IO int32_t leftTotalCounter = 0;
#define Right_Count                  ((int32_t)(0xFFFF))
__IO int32_t rightCounter = 0;
__IO int32_t rightValue = 0;
__IO int32_t rightTotalCounter = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void TIM1_Config(void);	// TIM1= Left
static void TIM3_Config(void);	// TIM8= Right
static void LED_Config(void);
static void BUTTON_Config(void);
//static void UU_PutString(uint8_t * str); // ok, but don't use this function
static void UU_PutNumber(int32_t leftC, int32_t rightC);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void TIM3_IRQHandler(void){
 HAL_TIM_IRQHandler(&htimRight2);
}
/* USER CODE END 0 */

int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

	//BUTTON_Config();
  //LED_Config();
	
	TIM1_Config();
	TIM3_Config();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htimLeft, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htimRight2, TIM_CHANNEL_ALL);
  rightCounter--;		// not sure why rightCounter= 2 at starting point
  /* USER CODE END 2 */
	
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  uint8_t trBuffer[] = "vinh nguyen vinh nguyen vinh nguyen\n";

	HAL_Delay(1000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
		
  /* USER CODE BEGIN 3 */
		leftValue = TIM1->CNT;
		rightValue = TIM3->CNT;
		leftTotalCounter = (leftCounter - 1)*Left_Count + leftValue;
		rightTotalCounter = (rightCounter - 1)*Right_Count + rightValue;		
		UU_PutNumber(leftTotalCounter, rightTotalCounter);
		HAL_Delay(25);
  }
  /* USER CODE END 3 */

}
static void UU_PutNumber(int32_t leftC, int32_t rightC)
{
	uint8_t buffToSend[64];
	sprintf((char *)buffToSend, "%d%s%d%s", leftC,".",rightC,"\r\n");
	uint8_t j=strlen((const char*)buffToSend);
	CDC_Transmit_FS(buffToSend, j);
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
static void TIM1_Config(void){	// Left encoder
  
	__HAL_RCC_TIM1_CLK_ENABLE();
	
	TIM_Encoder_InitTypeDef encoderConfig;
	
	htimLeft.Instance = TIM1;
	htimLeft.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	//htimLeft.Init.CounterMode = TIM_COUNTERMODE_UP;
	htimLeft.Init.Period = 0xffff;
	htimLeft.Init.Prescaler = 0;
	htimLeft.Init.RepetitionCounter = 0;
	
	encoderConfig.EncoderMode = TIM_ENCODERMODE_TI2;
	encoderConfig.IC1Filter = 0;
	encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
	encoderConfig.IC2Filter = 0;
	//encoderConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
	encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	
	
	if(HAL_TIM_Encoder_Init(&htimLeft, &encoderConfig) != HAL_OK){
	  
		Error_Handler();
	}
	
	__HAL_TIM_ENABLE_IT(&htimLeft, TIM_IT_UPDATE);
	
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	
	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	
}

static void TIM3_Config(void){	// Right encoder
 
  TIM_Encoder_InitTypeDef encoder;
 HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
 HAL_NVIC_SetPriority(SysTick_IRQn, 0, 1);
 
 htimRight2.Instance = TIM3;
 htimRight2.Init.Period = 0xFFFF;
 //htimRight2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimRight2.Init.Prescaler = 0;
  htimRight2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htimRight2.Init.RepetitionCounter = 0;
	//--
	//--
 
 encoder.EncoderMode = TIM_ENCODERMODE_TI12;
 encoder.IC1Filter = 0;	//-- 0x0F;
 encoder.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE; //-- TIM_INPUTCHANNELPOLARITY_RISING;
 encoder.IC1Prescaler = TIM_ICPSC_DIV1;	//-- TIM_ICPSC_DIV4;
 encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
 
 encoder.IC2Filter = 0; 	//-- 0x0F;
 //encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
 encoder.IC2Prescaler = TIM_ICPSC_DIV1;	//-- TIM_ICPSC_DIV4;
 encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;	//-- TIM_ICSELECTION_DIRECTTI;
 
 if (HAL_TIM_Encoder_Init(&htimRight2, &encoder) != HAL_OK) {
  Error_Handler();
 }
 
 if(HAL_TIM_Encoder_Start_IT(&htimRight2,TIM_CHANNEL_1)!=HAL_OK){
  Error_Handler();
 }
 
	__HAL_TIM_ENABLE_IT(&htimRight2, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start(&htimRight2,TIM_CHANNEL_1);
	
	//HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
	//HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	//HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 2, 0);
	//HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
 
  TIM3->EGR = 1;           // Generate an update event
  TIM3->CR1 = 1;           // Enable the counter
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim){
  
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__GPIOE_CLK_ENABLE();
	
	if(htim->Instance == TIM1) {
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	}
	
	if (htim->Instance == TIM3) {
 
		GPIO_InitTypeDef GPIO_InitStruct;
 __TIM3_CLK_ENABLE();
 
 __GPIOB_CLK_ENABLE();
 
 GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
 GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL; //-- GPIO_PULLUP;
 GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
 GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
 HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
 
 HAL_NVIC_EnableIRQ(TIM3_IRQn);
 }
}

static void LED_Config(void){
  
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	if(htim->Instance == TIM1){
		if(TIM1->CR1 & TIM_CR1_DIR){
		  leftCounter--;
		}else{
		  leftCounter++;
		}
	}
	
	if(htim->Instance == TIM3){
	  //rightTotalCounter++;
		if(TIM3->CR1 & TIM_CR1_DIR){
		  rightCounter--;
		}else{
		  rightCounter++;
		}
	}
}

static void BUTTON_Config(void){
  
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  
	if(GPIO_Pin == GPIO_PIN_0){
	  
		leftTotalCounter = 0;
		rightTotalCounter = 0;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
