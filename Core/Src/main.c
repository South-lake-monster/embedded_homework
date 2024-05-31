/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void 	LCD_Test_Clear(void);			// 清屏测试
void 	LCD_Test_Variable (void);	   // 变量显示，包括整数和小数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  SPI_LCD_Init();    // 初始化LCD
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LCD_Test_Clear();    // 清屏测试
    LCD_Test_Variable();    // 变量显示测试
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*************************************************************************************************
*	�? �? �?:	LCD_Test_Clear
*
*	函数功能:	清屏测试
*************************************************************************************************/
void LCD_Test_Clear(void)
{
    uint8_t	i = 0;			// 计数变量
    
    LCD_SetDirection(Direction_H);		
    LCD_SetColor(LCD_DISP_BLACK);				// 设置画笔颜色

    for(i=0;i<5;i++)
    {
        switch (i)		// 切换背景�?
        {
            case 0: LCD_SetBackColor(LCD_DISP_RED); 		break;	
            case 1: LCD_SetBackColor(LCD_DISP_GREEN); 	break;				
            case 2: LCD_SetBackColor(LCD_DISP_BLUE); 		break;
            case 3: LCD_SetBackColor(LCD_DISP_GRAY); 		break;
            case 4: LCD_SetBackColor(LCD_DISP_WHITE); 		break;			
            default:	break;			
        }
        LCD_Clear();		// 清屏
        HAL_Delay(1000);
    }
}


/************************************************************************************************
*	�? �? �?:	LCD_Test_Variable
*
*	函数功能:	变量显示，包括整数和小数
*************************************************************************************************/
void LCD_Test_Variable (void)
{
    uint16_t i;					// 计数变量
    int32_t	a = 0;			// 定义整数变量，用于测�?
    int32_t	b = 0;			// 定义整数变量，用于测�?
    int32_t	c = 0;			// 定义整数变量，用于测�?

    double p = 123.123;	// 定义浮点数变量，用于测试
    double f = -123.123;	// 定义浮点数变量，用于测试
    
    LCD_SetBackColor(LCD_DISP_BLACK); 			//	设置背景�?
    LCD_Clear(); 								// 清屏
       
    LCD_SetColor(LCD_DISP_BLUE);					// 设置画笔，蓝绿色		
    LCD_DisplayString(0,30,"test +:");				
    LCD_DisplayString(0,60,"test -:");					
          
    LCD_SetColor(LCD_DISP_GREEN);				// 设置画笔，亮黄色		
    LCD_DisplayString(0,100,"test :");	
    LCD_DisplayString(0,130,"test0:");	
    
    LCD_SetColor(LCD_DISP_RED);					// 设置画笔	，亮红色		
    LCD_DisplayString(0,170,"test+.:");	
    LCD_DisplayString(0,200,"test-.:");		
    
    for(i=0;i<100;i++)
    {
        LCD_SetColor(LCD_DISP_BLUE);								// 设置画笔	，蓝绿色	
        LCD_ShowNumMode(Fill_Space);							// 多余位填充空�?
        LCD_DisplayNumber( 80,30, b+i*10, 4) ;				// 显示变量			
        LCD_DisplayNumber( 80,60, c-i*10, 4) ;				// 显示变量			
        
        LCD_SetColor(LCD_DISP_GREEN);								// 设置画笔，亮黄色	

        LCD_ShowNumMode(Fill_Space);								// 多余位填�? 空格
        LCD_DisplayNumber( 130,100, a+i*150, 8) ;				// 显示变量		

        LCD_ShowNumMode(Fill_Zero);								// 多余位填�?0      
        LCD_DisplayNumber( 130,130, b+i*150, 8) ;				// 显示变量			
        
        LCD_SetColor(LCD_DISP_RED);									// 设置画笔，亮红色			
        LCD_ShowNumMode(Fill_Space);								// 多余位填充空�?		
        LCD_DisplayDecimals( 100,170, p+i*0.1,  10,3);		// 显示小数	
        LCD_DisplayDecimals( 100,200, f+i*0.01, 10,4);		// 显示小数		
        
        HAL_Delay(15);				
    }
    HAL_Delay(2500);		
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
