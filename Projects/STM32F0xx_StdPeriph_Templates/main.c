/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.3.1
  * @date    17-January-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F0xx_StdPeriph_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{

	/*!< At this stage the microcontroller clock setting is already configured, 
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f0xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f0xx.c file
	 */ 
	  
	/* Add your application code here
	 */

	USART_Config();    
	SysTickTimer_Config();       

	ADC_DMA_Config();

	TIM_Config();

	EXTI_Config();

	SPI_Config();
//	SPI_CMD_EXAMPLE(AudioControl_SPI_CMD01);
	
	Flash_Erase(FLASH_USER_START_ADDR,FLASH_USER_END_ADDR);
	Flash_Write(FLASH_USER_START_ADDR,FLASH_USER_END_ADDR,DATA_32);
	Flash_Read(FLASH_USER_START_ADDR,FLASH_USER_END_ADDR,DATA_32);
	Flash_MEMTEST(NULL,NULL);
	
	/* Infinite loop */
	while (1)
	{
//		printf("111\r\n");
//		Delay_ms(3000);

//		ADC_ConversionOutput();

		USART_TX_Polling();
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
