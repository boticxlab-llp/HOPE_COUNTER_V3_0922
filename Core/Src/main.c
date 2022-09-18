/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "../Lib/lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
 void lcd_default(void);
 void print_value(int val);
 void lcd_print(void);
 void lcd_print_data(void);

 int net = 0;
 uint32_t total = 0;
 uint16_t in = 0, out = 0;

 uint32_t t1 = 0, t0 = 0;
 uint32_t timeout = 700;

 const char title[50] = "HOPE SEC EQP PVT LTD";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

const LCD16x2_CfgType LCD16x2_CfgParam =
{
	GPIOA,
	GPIO_PIN_12,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_3,
	GPIO_PIN_8,
	GPIO_PIN_9,
	2
};

const LCD16x2_CfgType LCD16x2_CfgParam;


void delay_ms (uint16_t ms)
{
	__HAL_TIM_SET_COUNTER(&htim14,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim14) < ms);  // wait for the counter to reach the us input in the parameter
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_3) // If The INT Source Is EXTI Line9 (A9 Pin)
    {

    	  static uint32_t last_interrupt_time = 0;
    	  uint32_t interrupt_time = HAL_GetTick(); //millis();

    	  // If interrupts come faster than 200ms, assume it's a bounce and ignore
    	  //   noInterrupts();
    	  if ((interrupt_time - last_interrupt_time) > timeout)
    	  {
    	    t1 = HAL_GetTick(); //millis();
    	  }
    	  last_interrupt_time = interrupt_time;
    }

    if(GPIO_Pin == GPIO_PIN_4) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	  static uint32_t last_interrupt_time = 0;
    	  uint32_t interrupt_time = HAL_GetTick();  //millis();
    	  // noInterrupts();
    	  // If interrupts come faster than 200ms, assume it's a bounce and ignore
    	  if ((interrupt_time - last_interrupt_time) > timeout) // && (int0_flag == 0
    	  {
    	    t0 = HAL_GetTick(); //millis();
    	  }
    	  last_interrupt_time = interrupt_time;
    }
}


/************************** LCD function *****************************/
void print_value(int val)
{
  char buff[50];
  sprintf(buff, "%06d", val);
  LCD_Write_String(buff);
}

void lcd_print_data()
{
  net = in - out;

//  if(net < 0)
//  {
//	  net = 0;
//  }
  total = in + out;
  // lcd.clear();
  LCD_Set_Cursor(5, 1);
  // LCD_Write_String(in);
  print_value(in);

  LCD_Set_Cursor(5, 2);
  // LCD_Write_String(out);
  print_value(out);

  //  count= count_in-count_out;
  LCD_Set_Cursor(5, 3);
  print_value(net);

  LCD_Set_Cursor(14, 3);
  print_value(total);
}

void lcd_print()
{
  net = in - out;
  total = in + out;
  LCD_Clear();
  LCD_Set_Cursor(0, 0);
  // lcd.scrollDisplayLeft(&title[0]);
  LCD_Write_String("HOPE SEC EQP PVT LTD");
  LCD_Set_Cursor(0, 1);
  LCD_Write_String("IN ");
  LCD_Set_Cursor(5, 1);
  //  LCD_Write_String(in);
  print_value(in);
  LCD_Set_Cursor(0, 2);
  LCD_Write_String("OUT ");
  LCD_Set_Cursor(5, 2);
  // LCD_Write_String(out);
  print_value(out);
  LCD_Set_Cursor(0, 3);
  LCD_Write_String("NET ");
  LCD_Set_Cursor(5, 3);
  print_value(net);
  // LCD_Write_String(0);

  LCD_Set_Cursor(14, 2);
  LCD_Write_String("TOTAL");

  LCD_Set_Cursor(14, 3);
  print_value(total);
  // LCD_Write_String(0);
}

void lcd_default()
{

//  lcd.begin(20, 4);
  LCD_Set_Cursor(0, 0);
  // lcd.scrollDisplayLeft(&title[0]);
  LCD_Write_String("HOPE SEC EQP PVT LTD");
  LCD_Set_Cursor(0, 1);
  LCD_Write_String("IN ");
  LCD_Set_Cursor(5, 1);
  // LCD_Write_String(0);
  print_value(0);
  LCD_Set_Cursor(0, 2);
  LCD_Write_String("OUT ");
  LCD_Set_Cursor(5, 2);
  // LCD_Write_String(0);
  print_value(0);
  LCD_Set_Cursor(0, 3);
  LCD_Write_String("NET ");
  LCD_Set_Cursor(5, 3);
  // LCD_Write_String(0);
  print_value(0);

  LCD_Set_Cursor(15, 2);
  LCD_Write_String("TOTAL");

  LCD_Set_Cursor(14, 3);
  // LCD_Write_String(0);
  print_value(0);
}



/****************************** end of code *****************/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM14_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim14); // START TIMER
//  uint8_t Test[] = ".............Setup Ready........... !!!\r\n"; //Data to send
//  HAL_UART_Transmit(&huart1,Test,sizeof(Test),10);// Sending in normal mode
//  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  LCD_Init();
  LCD_Clear();

  LCD_Set_Cursor(5, 0);
  LCD_Write_String("BOOTING");

  LCD_Set_Cursor(5, 1);
  LCD_Write_String("PLEASE");

  LCD_Set_Cursor(5, 2);
  LCD_Write_String("WAIT");
  delay_ms(7000);
  lcd_default();
  /* USER CODE END 2 */
uint8_t flag = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(flag)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		  flag = 0;
	  }
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);

//	  HAL_Delay(500);
	  if ((HAL_GetTick() - t1) < timeout && (HAL_GetTick() - t0) < timeout)
	  {
	    if (t1 > t0)
	    {
	      in++;
//	      Serial.print("IN :");
//	      Serial.println(in);
	      t0 = 0;
	      t1 = 0;
	      flag = 1;
	    }

	    else
	    {
	      out++;

//	      Serial.print("OUT :");
//	      Serial.println(out);
	      t0 = 0;
	      t1 = 0;
	      flag = 1;
	    }
	  }

	  lcd_print_data();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_Pin|RS_Pin|D5_Pin|D6_Pin
                          |D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D7_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin RS_Pin D5_Pin D6_Pin
                           D4_Pin */
  GPIO_InitStruct.Pin = EN_Pin|RS_Pin|D5_Pin|D6_Pin
                          |D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin PB5 */
  GPIO_InitStruct.Pin = D7_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
