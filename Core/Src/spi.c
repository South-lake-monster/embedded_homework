/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
#define  LCD_SPI 	hspi3           // SPI局部宏，方便修改和移植

static pFONT	*LCD_AsciiFonts;		// 英文字体，ASCII字符集

// 因为这类SPI的屏幕，每次更新显示时，需要先配置坐标区域、再写显存，
// 在显示字符时，如果是一个个点去写坐标写显存，会非常慢，
// 因此开辟一片缓冲区，先将需要显示的数据写进缓冲区，最后再批量写入显存。
// 用户可以根据实际情况去修改此处缓冲区的大小，
// 例如，用户需要显示32*32的汉字时，需要的大小为 32*32*2 = 2048 字节（每个像素点占2字节）
uint16_t  LCD_Buff[DMA_SPI_BUF_SIZE];        // LCD缓冲区，16位宽（每个像素点占2字节）

struct	LCD_Struct//LCD相关参数结构体
{
	uint16_t Color;  				//	LCD当前画笔颜色
	uint16_t BackColor;			//	背景色
	uint8_t  ShowNum_Mode;		// 数字显示模式
	uint8_t  Direction;			//	显示方向
	uint16_t Width;            // 屏幕像素长度
	uint16_t Height;           // 屏幕像素宽度	
	uint8_t  X_Offset;         // X坐标偏移，用于设置屏幕控制器的显存写入方式
	uint8_t  Y_Offset;         // Y坐标偏移，用于设置屏幕控制器的显存写入方式
};
struct LCD_Struct LCD;

volatile uint8_t sendFlg = 0;	// DMA传输完成标志
/* USER CODE END 0 */

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

/* SPI3 init function */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */
    GPIO_LDC_Backlight_CLK_ENABLE;   // 使能 背光        引脚时钟
    GPIO_LDC_DC_CLK_ENABLE;          // 使能 数据指令选择 引脚时钟
  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PA4     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = SPI3_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI3_SCL_Pin|SPI3_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SPI3 DMA Init */
    /* SPI3_TX Init */
    hdma_spi3_tx.Instance = DMA1_Stream5;
    hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi3_tx);

  /* USER CODE BEGIN SPI3_MspInit 1 */
    // 初始化 背光 引脚  
		GPIO_InitStruct.Pin 		= LCD_Backlight_PIN;				// 背光 引脚
		GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;			// 推挽输出模式
		GPIO_InitStruct.Pull 	= GPIO_PULLDOWN;					// 下拉，默认保持低电平
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;			// 速度等级低
		HAL_GPIO_Init(LCD_Backlight_PORT, &GPIO_InitStruct);	// 初始化  

    // 初始化 数据指令选择 引脚  
		GPIO_InitStruct.Pin 		= LCD_DC_PIN;				      // 数据指令选择 引脚
		GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;			// 推挽输出模式
		GPIO_InitStruct.Pull 	= GPIO_NOPULL;						// 无上下拉
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;			// 速度等级低
		HAL_GPIO_Init(LCD_DC_PORT, &GPIO_InitStruct);	      // 初始化 
  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PA4     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(SPI3_CS_GPIO_Port, SPI3_CS_Pin);

    HAL_GPIO_DeInit(GPIOC, SPI3_SCL_Pin|SPI3_SDA_Pin);

    /* SPI3 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmatx);
  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*****************************************************************************************
*	函 数 名: LCD_WriteCMD
*	入口参数: CMD - 需要写入的控制指令
*	返 回 值: 无
*	函数功能: 用于写入控制字
*	说    明: 无
******************************************************************************************/
void  LCD_WriteCommand(uint8_t lcd_command)
{
   LCD_DC_Command;     // 数据指令选择 引脚输出低电平，代表本次传输 指令

   HAL_SPI_Transmit(&LCD_SPI, &lcd_command, 1, 1000) ;
}

/****************************************************************************************************************************************
*	函 数 名: LCD_WriteData_8bit
*
*	入口参数: lcd_data - 需要写入的数据，8位
*
*	函数功能: 写入8位数据
*	
****************************************************************************************************************************************/
void  LCD_WriteData_8bit(uint8_t lcd_data)
{
   LCD_DC_Data;     // 数据指令选择 引脚输出高电平，代表本次传输 数据

   HAL_SPI_Transmit(&LCD_SPI, &lcd_data, 1, 1000) ; // 启动SPI传输
}

/****************************************************************************************************************************************
*	函 数 名: LCD_WriteData_16bit
*
*	入口参数: lcd_data - 需要写入的数据，16位
*
*	函数功能: 写入16位数据
*	
****************************************************************************************************************************************/
void  LCD_WriteData_16bit(uint16_t lcd_data)
{
   uint8_t lcd_data_buff[2];    // 数据发送区
   lcd_data_buff[0] = lcd_data>>8;  // 将数据拆分
   lcd_data_buff[1] = lcd_data;
   LCD_DC_Data;      // 数据指令选择 引脚输出高电平，代表本次传输 数据
 
	HAL_SPI_Transmit_DMA(&LCD_SPI, lcd_data_buff, 2);	// 启动SPI传输
	WaitSendOver();
}

/****************************************************************************************************************************************
*	函 数 名: LCD_WriteBuff
*
*	入口参数: DataBuff - 数据区，DataSize - 数据长度
*
*	函数功能: 批量写入数据到屏幕
*	
****************************************************************************************************************************************/
void  LCD_WriteBuff(uint16_t *DataBuff, uint16_t DataSize)
{
	LCD_DC_Data;     // 数据指令选择 引脚输出高电平，代表本次传输 数据	
	HAL_SPI_Transmit_DMA(&LCD_SPI, (uint8_t *)DataBuff, DataSize*2);	// 启动SPI传输
	WaitSendOver();
}

/****************************************************************************************************************************************
*	函 数 名: SPI_LCD_Init
*
*	函数功能: 初始化SPI以及屏幕控制器的各种参数
*	
****************************************************************************************************************************************/
void SPI_LCD_Init(void)
{
   	MX_SPI3_Init();               // 初始化SPI和控制引脚
    HAL_Delay(10);               // 屏幕刚完成复位时（包括上电复位），需要等待5ms才能发送指令

 	LCD_WriteCommand(0x36);       // 显存访问控制 指令，用于设置访问显存的方式
	LCD_WriteData_8bit(0x00);     // 配置成 从上到下、从左到右，RGB像素格式

	LCD_WriteCommand(0x3A);			// 接口像素格式 指令，用于设置使用 12位、16位还是18位色
	LCD_WriteData_8bit(0x05);     // 此处配置成 16位 像素格式

	// 接下来很多都是电压设置指令，直接使用厂家给设定值
 	LCD_WriteCommand(0xB2);			
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x0C); 
	LCD_WriteData_8bit(0x00); 
	LCD_WriteData_8bit(0x33); 
	LCD_WriteData_8bit(0x33); 			

	LCD_WriteCommand(0xB7);		   // 栅极电压设置指令	
	LCD_WriteData_8bit(0x35);     // VGH = 13.26V，VGL = -10.43V

	LCD_WriteCommand(0xBB);			// 公共电压设置指令
	LCD_WriteData_8bit(0x19);     // VCOM = 1.35V

	LCD_WriteCommand(0xC0);
	LCD_WriteData_8bit(0x2C);

	LCD_WriteCommand(0xC2);       // VDV 和 VRH 来源设置
	LCD_WriteData_8bit(0x01);     // VDV 和 VRH 由用户自由配置

	LCD_WriteCommand(0xC3);			// VRH电压 设置指令  
	LCD_WriteData_8bit(0x12);     // VRH电压 = 4.6+( vcom+vcom offset+vdv)
				
	LCD_WriteCommand(0xC4);		   // VDV电压 设置指令	
	LCD_WriteData_8bit(0x20);     // VDV电压 = 0v

	LCD_WriteCommand(0xC6); 		// 正常模式的帧率控制指令
	LCD_WriteData_8bit(0x0F);   	// 设置屏幕控制器的刷新帧率为60帧    

	LCD_WriteCommand(0xD0);			// 电源控制指令
	LCD_WriteData_8bit(0xA4);     // 无效数据，固定写入0xA4
	LCD_WriteData_8bit(0xA1);     // AVDD = 6.8V ，AVDD = -4.8V ，VDS = 2.3V

	LCD_WriteCommand(0xE0);       // 正极电压伽马值设定
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2B);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x54);
	LCD_WriteData_8bit(0x4C);
	LCD_WriteData_8bit(0x18);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x0B);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x23);

	LCD_WriteCommand(0xE1);      // 负极电压伽马值设定
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2C);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x44);
	LCD_WriteData_8bit(0x51);
	LCD_WriteData_8bit(0x2F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x20);
	LCD_WriteData_8bit(0x23);

	LCD_WriteCommand(0x21);       // 打开反显，因为面板是常黑型，操作需要反过来

 	// 退出休眠指令，LCD控制器在刚上电、复位时，会自动进入休眠模式 ，因此操作屏幕之前，需要退出休眠  
	LCD_WriteCommand(0x11);       // 退出休眠 指令
   	HAL_Delay(120);               // 需要等待120ms，让电源电压和时钟电路稳定下来

 	// 打开显示指令，LCD控制器在刚上电、复位时，会自动关闭显示 
	LCD_WriteCommand(0x29);       // 打开显示   	
	
	// 以下进行一些驱动的默认设置
   	LCD_SetDirection(Direction_H);  	      //设置显示方向
	LCD_SetBackColor(LCD_DISP_BLACK);           // 设置背景色
 	LCD_SetColor(LCD_DISP_WHITE);             // 设置画笔色  
	LCD_Clear();                          	// 清屏

   	LCD_SetAsciiFont(&ASCII_Font24);       // 设置默认字体
   	LCD_ShowNumMode(Fill_Zero);	      	// 设置变量显示模式，多余位填充空格还是填充0

	// 全部设置完毕之后，打开背光	
   	LCD_Backlight_ON;  // 引脚输出高电平点亮背光
}

/****************************************************************************************************************************************
*	函 数 名:	 LCD_SetAddress
*
*	入口参数:	 x1 - 起始水平坐标   y1 - 起始垂直坐标  
*              x2 - 终点水平坐标   y2 - 终点垂直坐标	   
*	
*	函数功能:   设置需要显示的坐标区域		 			 
*****************************************************************************************************************************************/
void LCD_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)		
{
	LCD_WriteCommand(0x2a);			//	列地址设置，即X坐标
	LCD_WriteData_16bit(x1+LCD.X_Offset);
	LCD_WriteData_16bit(x2+LCD.X_Offset);

	LCD_WriteCommand(0x2b);			//	行地址设置，即Y坐标
	LCD_WriteData_16bit(y1+LCD.Y_Offset);
	LCD_WriteData_16bit(y2+LCD.Y_Offset);

	LCD_WriteCommand(0x2c);			//	开始写入显存，即要显示的颜色数据
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_SetColor
*
*	入口参数:	Color - 要显示的颜色，示例：0x0000FF 表示蓝色
*
*	函数功能:	此函数用于设置画笔的颜色，例如显示字符、画点画线、绘图的颜色
*
*	说    明:	1. 为了方便用户使用自定义颜色，入口参数 Color 使用24位 RGB888的颜色格式，用户无需关心颜色格式的转换
*					2. 24位的颜色中，从高位到低位分别对应 R、G、B  3个颜色通道
*
*****************************************************************************************************************************************/
void LCD_SetColor(uint16_t Color)
{
	LCD.Color = Color;  // 将颜色写入全局LCD参数		
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_SetBackColor
*
*	入口参数:	Color - 要显示的颜色，示例：0x0000FF 表示蓝色
*
*	函数功能:	设置背景色,此函数用于清屏以及显示字符的背景色
*
*	说    明:	1. 为了方便用户使用自定义颜色，入口参数 Color 使用24位 RGB888的颜色格式，用户无需关心颜色格式的转换
*					2. 24位的颜色中，从高位到低位分别对应 R、G、B  3个颜色通道
*
*****************************************************************************************************************************************/
void LCD_SetBackColor(uint16_t Color)
{
	LCD.BackColor = Color;	// 将颜色写入全局LCD参数			   	
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_SetDirection
*
*	入口参数:	direction - 要显示的方向
*
*	函数功能:	设置要显示的方向
*
*	说    明:   1. 可输入参数 Direction_H 、Direction_V 、Direction_H_Flip 、Direction_V_Flip        
*              2. 使用示例 LCD_DisplayDirection(Direction_H) ，即设置屏幕横屏显示
*
*****************************************************************************************************************************************/
void LCD_SetDirection(uint8_t direction)
{
	LCD.Direction = direction;    // 写入全局LCD参数

   if( direction == Direction_H )   // 横屏显示
   {
      LCD_WriteCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
      LCD_WriteData_8bit(0x70);        // 横屏显示
      LCD.X_Offset   = 20;             // 设置控制器坐标偏移量
      LCD.Y_Offset   = 0;   
      LCD.Width      = LCD_Height;		// 重新赋值长、宽
      LCD.Height     = LCD_Width;		
   }
   else if( direction == Direction_V )
   {
      LCD_WriteCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
      LCD_WriteData_8bit(0x00);        // 垂直显示
      LCD.X_Offset   = 0;              // 设置控制器坐标偏移量
      LCD.Y_Offset   = 20;     
      LCD.Width      = LCD_Width;		// 重新赋值长、宽
      LCD.Height     = LCD_Height;						
   }
   else if( direction == Direction_H_Flip )
   {
      LCD_WriteCommand(0x36);   			 // 显存访问控制 指令，用于设置访问显存的方式
      LCD_WriteData_8bit(0xA0);         // 横屏显示，并上下翻转，RGB像素格式
      LCD.X_Offset   = 20;              // 设置控制器坐标偏移量
      LCD.Y_Offset   = 0;      
      LCD.Width      = LCD_Height;		 // 重新赋值长、宽
      LCD.Height     = LCD_Width;				
   }
   else if( direction == Direction_V_Flip )
   {
      LCD_WriteCommand(0x36);    		// 显存访问控制 指令，用于设置访问显存的方式
      LCD_WriteData_8bit(0xC0);        // 垂直显示 ，并上下翻转，RGB像素格式
      LCD.X_Offset   = 0;              // 设置控制器坐标偏移量
      LCD.Y_Offset   = 20;     
      LCD.Width      = LCD_Width;		// 重新赋值长、宽
      LCD.Height     = LCD_Height;				
   }   
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_SetAsciiFont
*
*	入口参数:	*fonts - 要设置的ASCII字体
*
*	函数功能:	设置ASCII字体，可选择使用 3216/2412/2010/1608/1206 五种大小的字体
*
*	说    明:	1. 使用示例 LCD_SetAsciiFont(&ASCII_Font24) ，即设置 2412的 ASCII字体
*					2. 相关字模存放在 lcd_fonts.c 			
*
*****************************************************************************************************************************************/
void LCD_SetAsciiFont(pFONT *Asciifonts)
{
  LCD_AsciiFonts = Asciifonts;
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_Clear
*
*	函数功能:	清屏函数，将LCD清除为 LCD.BackColor 的颜色
*
*	说    明:	先用 LCD_SetBackColor() 设置要清除的背景色，再调用该函数清屏即可
*
*****************************************************************************************************************************************/
void LCD_Clear(void)
{
	for(int i=0; i<DMA_SPI_BUF_SIZE; i++)
	{
		LCD_Buff[i] = LCD.BackColor;	// 使用背景色填充显存
	}
  	LCD_SetAddress(0,0,LCD.Width-1,LCD.Height-1);	// 设置坐标
	LCD_DC_Data;     // 数据指令选择 引脚输出高电平，代表本次传输 数据			
	for(int j=0; j<75; j++)
	{
		HAL_SPI_Transmit_DMA(&LCD_SPI, (uint8_t *)LCD_Buff, DMA_SPI_BUF_SIZE*2);	// 启动SPI传输
		WaitSendOver();
	}
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_ClearRect
*
*	入口参数:	x - 起始水平坐标
*					y - 起始垂直坐标
*					width  - 要清除区域的横向长度
*					height - 要清除区域的纵向宽度
*
*	函数功能:	局部清屏函数，将指定位置对应的区域清除为 LCD.BackColor 的颜色
*
*	说    明:	1. 先用 LCD_SetBackColor() 设置要清除的背景色，再调用该函数清屏即可
*				   2. 使用示例 LCD_ClearRect( 10, 10, 100, 50) ，清除坐标(10,10)开始的长100宽50的区域
*
*****************************************************************************************************************************************/
void LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
   LCD_SetAddress( x, y, x+width-1, y+height-1);	// 设置坐标	
	
	LCD_DC_Data;     // 数据指令选择 引脚输出高电平，代表本次传输 数据		
	
	HAL_SPI_Transmit_DMA(&LCD_SPI, (uint8_t *)LCD.BackColor, LCD.Width*LCD.Height*2);	// 启动SPI传输
	WaitSendOver();
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_DrawPoint
*
*	入口参数:	x - 起始水平坐标
*					y - 起始垂直坐标
*					color  - 要绘制的颜色，使用 24位 RGB888 的颜色格式，用户无需关心颜色格式的转换
*
*	函数功能:	在指定坐标绘制指定颜色的点
*
*	说    明:	使用示例 LCD_DrawPoint( 10, 10, 0x0000FF) ，在坐标(10,10)绘制蓝色的点
*
*****************************************************************************************************************************************/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_SetAddress(x,y,x,y);	//	设置坐标 

	LCD_WriteData_16bit(color)	;
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_DisplayChar
*
*	入口参数:	x - 起始水平坐标
*					y - 起始垂直坐标
*					c  - ASCII字符
*
*	函数功能:	在指定坐标显示指定的字符
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetAsciiFont(&ASCII_Font24) 设置为 2412的ASCII字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0xff0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0x000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayChar( 10, 10, 'a') ，在坐标(10,10)显示字符 'a'
*
*****************************************************************************************************************************************/
void LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t c)
{
	uint16_t  index = 0, counter = 0 ,i = 0, w = 0;		// 计数变量
   	uint8_t   disChar;		//存储字符的地址

	c = c - 32; 	// 计算ASCII字符的偏移

	for(index = 0; index < LCD_AsciiFonts->Sizes; index++)	
	{
		disChar = LCD_AsciiFonts->pTable[c*LCD_AsciiFonts->Sizes + index]; //获取字符的模值
		for(counter = 0; counter < 8; counter++)
		{ 
			if(disChar & 0x01)	
			{		
            LCD_Buff[i] =  LCD.Color;			// 当前模值不为0时，使用画笔色绘点
			}
			else		
			{		
            LCD_Buff[i] = LCD.BackColor;		//否则使用背景色绘制点
			}
			disChar >>= 1;
			i++;
         	w++;
 			if( w == LCD_AsciiFonts->Width ) // 如果写入的数据达到了字符宽度，则退出当前循环
			{								   // 进入下一字符的写入的绘制
				w = 0;
				break;
			}        
		}	
	}		
   LCD_SetAddress( x, y, x+LCD_AsciiFonts->Width-1, y+LCD_AsciiFonts->Height-1);	   // 设置坐标	
   LCD_WriteBuff(LCD_Buff,LCD_AsciiFonts->Width*LCD_AsciiFonts->Height);          // 写入显存
}

/****************************************************************************************************************************************
*	函 数 名:	LCD_DisplayString
*
*	入口参数:	x - 起始水平坐标
*					y - 起始垂直坐标
*					p - ASCII字符串的首地址
*
*	函数功能:	在指定坐标显示指定的字符串
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetAsciiFont(&ASCII_Font24) 设置为 2412的ASCII字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0x0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0x000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayString( 10, 10, "FANKE") ，在起始坐标为(10,10)的地方显示字符串"FANKE"
*
*****************************************************************************************************************************************/
void LCD_DisplayString( uint16_t x, uint16_t y, char *p) 
{  
	while ((x < LCD.Width) && (*p != 0))	//判断显示坐标是否超出显示区域并且字符是否为空字符
	{
		 LCD_DisplayChar( x,y,*p);
		 x += LCD_AsciiFonts->Width; //显示下一个字符
		 p++;	//取下一个字符地址
	}
}

/*****************************************************************************************************************************************
*	函 数 名:	LCD_ShowNumMode
*
*	入口参数:	mode - 设置变量的显示模式
*
*	函数功能:	设置变量显示时多余位补0还是补空格，可输入参数 Fill_Space 填充空格，Fill_Zero 填充零
*
*	说    明:   1. 只有 LCD_DisplayNumber() 显示整数 和 LCD_DisplayDecimals()显示小数 这两个函数用到
*					2. 使用示例 LCD_ShowNumMode(Fill_Zero) 设置多余位填充0，例如 123 可以显示为 000123
*
*****************************************************************************************************************************************/
void LCD_ShowNumMode(uint8_t mode)
{
	LCD.ShowNum_Mode = mode;
}

/*****************************************************************************************************************************************
*	函 数 名:	LCD_DisplayNumber
*
*	入口参数:	x - 起始水平坐标
*					y - 起始垂直坐标
*					number - 要显示的数字,范围在 -2147483648~2147483647 之间
*					len - 数字的位数，如果位数超过len，将按其实际长度输出，如果需要显示负数，请预留一个位的符号显示空间
*
*	函数功能:	在指定坐标显示指定的整数变量
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetAsciiFont(&ASCII_Font24) 设置为的ASCII字符字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0x0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0x000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayNumber( 10, 10, a, 5) ，在坐标(10,10)显示指定变量a,总共5位，多余位补0或空格，
*						例如 a=123 时，会根据 LCD_ShowNumMode()的设置来显示  123(前面两个空格位) 或者00123
*						
*****************************************************************************************************************************************/
void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number, uint8_t len) 
{  
	char   Number_Buffer[15];				// 用于存储转换后的字符串

	if( LCD.ShowNum_Mode == Fill_Zero)	// 多余位补0
	{
		sprintf( Number_Buffer , "%0.*d",len, number );	// 将 number 转换成字符串，便于显示		
	}
	else			// 多余位补空格
	{	
		sprintf( Number_Buffer , "%*d",len, number );	// 将 number 转换成字符串，便于显示		
	}
	
	LCD_DisplayString( x, y,(char *)Number_Buffer) ;  // 将转换得到的字符串显示出来
	
}

/***************************************************************************************************************************************
*	函 数 名:	LCD_DisplayDecimals
*
*	入口参数:	x - 起始水平坐标
*					y - 起始垂直坐标
*					decimals - 要显示的数字, double型取值1.7 x 10^（-308）~ 1.7 x 10^（+308），但是能确保准确的有效位数为15~16位
*
*       			len - 整个变量的总位数（包括小数点和负号），若实际的总位数超过了指定的总位数，将按实际的总长度位输出，
*							示例1：小数 -123.123 ，指定 len <=8 的话，则实际照常输出 -123.123
*							示例2：小数 -123.123 ，指定 len =10 的话，则实际输出   -123.123(负号前面会有两个空格位) 
*							示例3：小数 -123.123 ，指定 len =10 的话，当调用函数 LCD_ShowNumMode() 设置为填充0模式时，实际输出 -00123.123 
*
*					decs - 要保留的小数位数，若小数的实际位数超过了指定的小数位，则按指定的宽度四舍五入输出
*							 示例：1.12345 ，指定 decs 为4位的话，则输出结果为1.1235
*
*	函数功能:	在指定坐标显示指定的变量，包括小数
*
*	说    明:	1. 可设置要显示的字体，例如使用 LCD_SetAsciiFont(&ASCII_Font24) 设置为的ASCII字符字体
*					2.	可设置要显示的颜色，例如使用 LCD_SetColor(0x0000FF) 设置为蓝色
*					3. 可设置对应的背景色，例如使用 LCD_SetBackColor(0x000000) 设置为黑色的背景色
*					4. 使用示例 LCD_DisplayDecimals( 10, 10, a, 5, 3) ，在坐标(10,10)显示字变量a,总长度为5位，其中保留3位小数
*						
*****************************************************************************************************************************************/
void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double decimals, uint8_t len, uint8_t decs) 
{  
	char  Number_Buffer[20];				// 用于存储转换后的字符串
	
	if( LCD.ShowNum_Mode == Fill_Zero)	// 多余位填充0模式
	{
		sprintf( Number_Buffer , "%0*.*lf",len,decs, decimals );	// 将 number 转换成字符串，便于显示		
	}
	else		// 多余位填充空格
	{
		sprintf( Number_Buffer , "%*.*lf",len,decs, decimals );	// 将 number 转换成字符串，便于显示		
	}
	
	LCD_DisplayString( x, y,(char *)Number_Buffer) ;	// 将转换得到的字符串显示出来
}

/* USER CODE END 1 */
