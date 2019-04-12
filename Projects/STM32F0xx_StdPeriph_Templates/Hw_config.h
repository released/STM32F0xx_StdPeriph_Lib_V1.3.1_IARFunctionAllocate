/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <stdio.h>
#include "string.h"

/* Define config -------------------------------------------------------------*/
#define TRUE			1
#define FALSE           0
#define 	ON				1
#define 	OFF				0
typedef unsigned char   BOOL;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

#define FLASH_PAGE_SIZE         					((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   					((uint32_t)0x08005000)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     					((uint32_t)0x08006000)   /* End @ of user Flash area */
#define DATA_32                 					((uint32_t)0x12345678)
#define DATA_32r                 					((uint32_t)0x87654321)

#define ADC_Number 								100

#define AudioControl_ADC1                         	ADC1

#define AudioControl_ADC1_TEMP_1_CLK             	RCC_APB2Periph_ADC1
#define AudioControl_ADC1_TEMP_1_GPIO_PORT      	GPIOA	
#define AudioControl_ADC1_TEMP_1_GPIO_CLK       	RCC_AHBPeriph_GPIOA
#define AudioControl_ADC1_TEMP_1_PIN             	GPIO_Pin_1

#define AudioControl_ADC1_TEMP_2_CLK          		RCC_APB2Periph_ADC1
#define AudioControl_ADC1_TEMP_2_GPIO_PORT   		GPIOA	
#define AudioControl_ADC1_TEMP_2_GPIO_CLK     	RCC_AHBPeriph_GPIOA
#define AudioControl_ADC1_TEMP_2_PIN          		GPIO_Pin_2

#define AudioControl_ADC1_VERF_MV					3300	//3300

#define AudioControl_USART                         	USART1
#define AudioControl_USART_CLK                     	RCC_APB2Periph_USART1
#define AudioControl_USART_GPIO_PORT          		GPIOA
#define AudioControl_USART_GPIO_CLK         		RCC_AHBPeriph_GPIOA

#define AudioControl_USART_TX_PIN                	GPIO_Pin_9
#define AudioControl_USART_TX_SOURCE            	GPIO_PinSource9
#define AudioControl_USART_TX_AF                  	GPIO_AF_1

#define AudioControl_USART_RX_PIN                 	GPIO_Pin_10
#define AudioControl_USART_RX_SOURCE             	GPIO_PinSource10
#define AudioControl_USART_RX_AF                  	GPIO_AF_1

#define AudioControl_USART_IRQn                   	USART1_IRQn

#define AudioControl_T1_PIN                   		GPIO_Pin_0
#define AudioControl_T1_GPIO_PORT             		GPIOB
#define AudioControl_T1_GPIO_CLK              		RCC_AHBPeriph_GPIOB
#define AudioControl_T1_EXTI_LINE             		EXTI_Line0
#define AudioControl_T1_EXTI_PORT_SOURCE      		EXTI_PortSourceGPIOB
#define AudioControl_T1_EXTI_PIN_SOURCE       		EXTI_PinSource0
#define AudioControl_T1_EXTI_IRQn             		EXTI0_1_IRQn 

#define AudioControl_T2_PIN                   		GPIO_Pin_1
#define AudioControl_T2_GPIO_PORT             		GPIOB
#define AudioControl_T2_GPIO_CLK              		RCC_AHBPeriph_GPIOB
#define AudioControl_T2_EXTI_LINE             		EXTI_Line1
#define AudioControl_T2_EXTI_PORT_SOURCE      		EXTI_PortSourceGPIOB
#define AudioControl_T2_EXTI_PIN_SOURCE       		EXTI_PinSource1
#define AudioControl_T2_EXTI_IRQn             		EXTI0_1_IRQn 

#define AudioControl_SPI                         	SPI1
#define AudioControl_SPI_CLK                     	RCC_APB2Periph_SPI1
#define AudioControl_SPI_GPIO_PORT          		GPIOA|GPIOB
#define AudioControl_SPI_GPIO_CLK         			RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB

#define AudioControl_SPI_CLK_PORT                  	GPIOA
#define AudioControl_SPI_CLK_PIN                  	GPIO_Pin_5
#define AudioControl_SPI_CLK_SOURCE              	GPIO_PinSource5
#define AudioControl_SPI_CLK_AF                  	GPIO_AF_0

#define AudioControl_SPI_MISO_PORT                 GPIOA
#define AudioControl_SPI_MISO_PIN                  	GPIO_Pin_6
#define AudioControl_SPI_MISO_SOURCE              	GPIO_PinSource6
#define AudioControl_SPI_MISO_AF                  	GPIO_AF_0

#define AudioControl_SPI_MOSI_PORT                 GPIOA
#define AudioControl_SPI_MOSI_PIN                  	GPIO_Pin_7
#define AudioControl_SPI_MOSI_SOURCE              	GPIO_PinSource7
#define AudioControl_SPI_MOSI_AF                  	GPIO_AF_0

#define AudioControl_SPI_NSS_PORT                 	GPIOA
#define AudioControl_SPI_NSS_PIN                  	GPIO_Pin_4
#define AudioControl_SPI_NSS_SOURCE              	GPIO_PinSource4
#define AudioControl_SPI_NSS_AF                  	GPIO_AF_0

#define 	AudioControl_SPI_CMD01						0x70

extern __IO uint16_t CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern uint32_t Data;
/* Macro ---------------------------------------------------------------------*/
/*
#define UartTxPutChar(x)		\
{	\
     UART1_SendData8(x);	\
     while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	\
}*/

/* Exported types ------------------------------------------------------------*/
void TIM_Config(void);
void EXTI_Config(void);

void	 ADC_ConversionOutput(void);
void ADC_DMA_Config(void);

void Flash_Erase(uint32_t start , uint32_t end) @".Flash_Erase";
void Flash_Write(uint32_t start , uint32_t end , uint32_t DataWrite) @".Flash_Write";
void Flash_Read(uint32_t start , uint32_t end , uint32_t DataRead) @".Flash_Read";
void Flash_MEMTEST(uint32_t StartAddress , uint32_t Length);

void SPI_CMD_EXAMPLE(uint8_t Cmd);
void SPI_Config(void);

void USART_TX_Polling(void);
void USART_TEST(void);
void USART_Config(void);  

void SysTickTimer_Config(void);

void Delay_ms(__IO uint32_t uTime);
void Delay_s(__IO uint32_t mTime);

void TimingDelay_Decrement(void);
void UART_SendByte(uint8_t Data);
void UART_SendString(uint8_t* Data,uint16_t len);
void SystemClkDelay(uint32_t u32Delay);
/* Exported constants --------------------------------------------------------*/

#endif  /* __HW_CONFIG_H */

