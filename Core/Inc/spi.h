/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "usart.h"
#include <math.h>
#include "lcd_fonts.h"
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN Private defines */
/*----------------------------------------------- 参数�? -------------------------------------------*/
#define LCD_Width         240		// LCD的像素长�?
#define LCD_Height        320		// LCD的像素宽�?
#define DMA_SPI_BUF_SIZE  1024	// DMA传输缓存大小  每两个字节显示一个像�?

// 显示方向参数
// 使用示例：LCD_DisplayDirection(Direction_H) 设置屏幕横屏显示
#define	Direction_H			    0					//LCD横屏显示
#define	Direction_H_Flip	  1					//LCD横屏显示,上下翻转
#define	Direction_V				  2					//LCD竖屏显示 
#define	Direction_V_Flip	  3					//LCD竖屏显示,上下翻转 

// 设置变量显示时多余位�?0还是补空�?
// 只有 LCD_DisplayNumber() 显示整数 �? LCD_DisplayDecimals()显示小数 这两个函数用�?
// 使用示例�? LCD_ShowNumMode(Fill_Zero) 设置多余位填�?0，例�? 123 可以显示�? 000123
#define  Fill_Zero          0		//填充0
#define  Fill_Space         1		//填充空格

extern volatile uint8_t sendFlg;	// DMA传输完成标志
#define WaitSendOver()  while(sendFlg==0); sendFlg=0;	// 等待DMA传输完成

/*---------------------------------------- 常用颜色 ------------------------------------------------------

 1. 这里为了方便用户使用，定义的�?24�? RGB888颜色，然后再通过代码自动转换�? 16�? RGB565 的颜�?
 2. 24位的颜色中，从高位到低位分别对应 R、G、B  3个颜色�?�道
 3. 用户可以在电脑用调色板获�?24位RGB颜色，再将颜色输入LCD_SetColor()或LCD_SetBackColor()就可以显示出相应的颜�? 
 */                                                  						

/************** 颜色(RGB 5,6,5) **************/     //先高字节，后低字�? 
#define LCD_DISP_RED                    0xF800    // 红色                
#define LCD_DISP_YELLOW                 0xFFE0    // 黄色
#define LCD_DISP_GREEN                  0x07E0    // 绿色
#define LCD_DISP_BLUE                   0x001F    // 蓝色
#define LCD_DISP_WHITE                  0xFFFF    // 白色
#define LCD_DISP_BLACK                  0x0000    // 黑色
#define LCD_DISP_GRAY                   0xEF5D    // 灰色
#define LCD_DISP_GRAY75                 0x39E7    // 灰色75%
#define LCD_DISP_GRAY50                 0x7BEF    // 灰色50%
#define LCD_DISP_GRAY25                 0xADB5    // 灰色25%
#define LCD_DISP_BROWN                  0XBC40    // 棕色
#define LCD_DISP_ORANGE                 0xFD20    // 橙色
#define LCD_DISP_CYAN                   0x07FF    // 青色
#define LCD_DISP_MAGENTA                0xF81F    // 紫色
#define LCD_DISP_PINK                   0xFC18    // 粉色
#define LCD_DISP_SKYBLUE                0x867D    // 天蓝�?

 /*--------------------------------------------- LCD其它引脚 -----------------------------------------------*/
#define   LCD_Backlight_PIN								    SPI3_BL_Pin				         // 背光  引脚				
#define	  LCD_Backlight_PORT						  	  SPI3_BL_GPIO_Port									// 背光 GPIO端口
#define 	GPIO_LDC_Backlight_CLK_ENABLE       __HAL_RCC_GPIOE_CLK_ENABLE()	// 背光 GPIO时钟 	

#define	  LCD_Backlight_OFF		HAL_GPIO_WritePin(LCD_Backlight_PORT, LCD_Backlight_PIN, GPIO_PIN_RESET);	// 低电平，关闭背光
#define 	LCD_Backlight_ON		HAL_GPIO_WritePin(LCD_Backlight_PORT, LCD_Backlight_PIN, GPIO_PIN_SET);		// 高电平，�?启背�?
 
#define   LCD_DC_PIN						    SPI3_DC_Pin				         // 数据指令选择  引脚				
#define 	LCD_DC_PORT						    SPI3_DC_GPIO_Port									// 数据指令选择  GPIO端口
#define 	GPIO_LDC_DC_CLK_ENABLE    __HAL_RCC_GPIOE_CLK_ENABLE()	// 数据指令选择  GPIO时钟 	

#define	  LCD_DC_Command		    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);	   // 低电平，指令传输 
#define 	LCD_DC_Data		        HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);		// 高电平，数据传输
/* USER CODE END Private defines */

void MX_SPI3_Init(void);

/* USER CODE BEGIN Prototypes */
/*------------------------------------------------ 函数声明 ----------------------------------------------*/
void  SPI_LCD_Init(void);      // 液晶屏以及SPI初始�?   
void  LCD_Clear(void);			 // 清屏函数
void  LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);	// �?部清屏函�?

void  LCD_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);	// 设置坐标		
void  LCD_SetColor(uint16_t Color); 				   //	设置画笔颜色
void  LCD_SetBackColor(uint16_t Color);  				//	设置背景颜色
void  LCD_SetDirection(uint8_t direction);  	      //	设置显示方向

//>>>>>	显示ASCII字符
void  LCD_SetAsciiFont(pFONT *fonts);										//	设置ASCII字体
void 	LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t c);				//	显示单个ASCII字符
void 	LCD_DisplayString( uint16_t x, uint16_t y, char *p);	 		//	显示ASCII字符�?

//>>>>>	显示整数或小�?
void  LCD_ShowNumMode(uint8_t mode);		// 设置变量显示模式，多余位填充空格还是填充0
void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number,uint8_t len) ;					// 显示整数
void  LCD_DisplayDecimals( uint16_t x, uint16_t y, float number,uint8_t len,uint8_t decs);	// 显示小数
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

