/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "spi.h"
#include "motor.h"
#include "bsp_as5600.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//任务优先级
#define LCDVIEW_TASK_PRIO	1
//任务堆栈大小	
#define LCDVIEW_STK_SIZE 	256  
//任务句柄
TaskHandle_t LCDViewTask_Handler;
//任务函数
void LCDViewTask(void *pvParameters);

//任务优先级
#define MOTOR_TASK_PRIO	3
//任务堆栈大小	
#define MOTOR_STK_SIZE 	256  
//任务句柄
TaskHandle_t MotorTask_Handler;
//任务函数
void MotorTask(void *pvParameters);

//任务优先级
#define HALL_TASK_PRIO	2
//任务堆栈大小	
#define HALL_STK_SIZE 	256  
//任务句柄
TaskHandle_t HallTask_Handler;
//任务函数
void HallTask(void *pvParameters);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void 	LCD_Test_Clear(void);			// 清屏测试
void 	LCD_Test_Variable (void);	   // 变量显示，包括整数和小数
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float angle;
int angle_int, angle_frac;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for startTask */
osThreadId_t startTaskHandle;
const osThreadAttr_t startTask_attributes = {
  .name = "startTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of startTask */
  startTaskHandle = osThreadNew(StartTask, NULL, &startTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask */
/**
  * @brief  Function implementing the startTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask */
void StartTask(void *argument)
{
  /* USER CODE BEGIN StartTask */
  taskENTER_CRITICAL();           //进入临界�??
	//创建LCD的显示任务
  xTaskCreate((TaskFunction_t )LCDViewTask,             
              (const char*    )"LCDViewTask",           
              (uint16_t       )LCDVIEW_STK_SIZE,        
              (void*          )NULL,                  
              (UBaseType_t    )LCDVIEW_TASK_PRIO,        
              (TaskHandle_t*  )&LCDViewTask_Handler);   	

  //创建电机控制任务
  xTaskCreate((TaskFunction_t )MotorTask,             
              (const char*    )"MotorTask",           
              (uint16_t       )MOTOR_STK_SIZE,        
              (void*          )NULL,                  
              (UBaseType_t    )MOTOR_TASK_PRIO,        
              (TaskHandle_t*  )&MotorTask_Handler); 
  //创建霍尔传感器任务
  xTaskCreate((TaskFunction_t )HallTask,             
              (const char*    )"HallTask",           
              (uint16_t       )HALL_STK_SIZE,        
              (void*          )NULL,                  
              (UBaseType_t    )HALL_TASK_PRIO,        
              (TaskHandle_t*  )&HallTask_Handler); 
      
  osThreadTerminate(startTaskHandle); //删除�?始任�?
  taskEXIT_CRITICAL();            //�?出临界区
  /* USER CODE END StartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void LCDViewTask(void *pvParameters)
{
  while(1)
  {
    LCD_Test_Clear();    // 清屏测试
    LCD_Test_Variable();    // 变量显示测试
  }
}

void MotorTask(void *pvParameters)
{
  while(1)
  {
    velocityOpenloop(10);
    vTaskDelay(1);
  }
}

void HallTask(void *pvParameters)
{
  while(1)
  {
    angle = bsp_as5600GetAngle();
    angle_int = (int)angle;
    angle_frac = (int)((angle - angle_int) * 1000);
    Usart1Printf("angle: %d.%03d \r\n", angle_int, angle_frac);
    vTaskDelay(100);
  }
}
/*************************************************************************************************
*	�????? �????? �?????:	LCD_Test_Clear
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
        switch (i)		// 切换背景�?????
        {
            case 0: LCD_SetBackColor(LCD_DISP_RED); 		break;	
            case 1: LCD_SetBackColor(LCD_DISP_GREEN); 	break;				
            case 2: LCD_SetBackColor(LCD_DISP_BLUE); 		break;
            case 3: LCD_SetBackColor(LCD_DISP_GRAY); 		break;
            case 4: LCD_SetBackColor(LCD_DISP_WHITE); 		break;			
            default:	break;			
        }
        LCD_Clear();		// 清屏
        vTaskDelay(1000);
    }
}


/************************************************************************************************
*	�????? �????? �?????:	LCD_Test_Variable
*
*	函数功能:	变量显示，包括整数和小数
*************************************************************************************************/
void LCD_Test_Variable (void)
{
    uint16_t i;					// 计数变量
    int32_t	a = 0;			// 定义整数变量，用于测�?????
    int32_t	b = 0;			// 定义整数变量，用于测�?????
    int32_t	c = 0;			// 定义整数变量，用于测�?????

    double p = 123.123;	// 定义浮点数变量，用于测试
    double f = -123.123;	// 定义浮点数变量，用于测试
    
    LCD_SetBackColor(LCD_DISP_BLACK); 			//	设置背景�?????
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
        LCD_ShowNumMode(Fill_Space);							// 多余位填充空�?????
        LCD_DisplayNumber( 80,30, b+i*10, 4) ;				// 显示变量			
        LCD_DisplayNumber( 80,60, c-i*10, 4) ;				// 显示变量			
        
        LCD_SetColor(LCD_DISP_GREEN);								// 设置画笔，亮黄色	

        LCD_ShowNumMode(Fill_Space);								// 多余位填�????? 空格
        LCD_DisplayNumber( 130,100, a+i*150, 8) ;				// 显示变量		

        LCD_ShowNumMode(Fill_Zero);								// 多余位填�?????0      
        LCD_DisplayNumber( 130,130, b+i*150, 8) ;				// 显示变量			
        
        LCD_SetColor(LCD_DISP_RED);									// 设置画笔，亮红色			
        LCD_ShowNumMode(Fill_Space);								// 多余位填充空�?????		
        LCD_DisplayDecimals( 100,170, p+i*0.1,  10,3);		// 显示小数	
        LCD_DisplayDecimals( 100,200, f+i*0.01, 10,4);		// 显示小数		
        
        vTaskDelay(15);				
    }
    vTaskDelay(2500);		
}
/* USER CODE END Application */

