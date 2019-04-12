/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.3.1
  * @date    17-January-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f0xx_it.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t capture = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{    
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		//Insert TIM3 function		
//		printf("TIM3_IRQHandler (1ms) \r\n");
		if (GPIO_ReadInputDataBit(AudioControl_T1_GPIO_PORT ,AudioControl_T1_PIN)== TRUE)
		{
			printf("1111111111!\r\n");	
		}
		
		capture = TIM_GetCapture1(TIM3);
		TIM_SetCompare1(TIM3, capture + CCR1_Val);		
	}
	else
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		//Insert TIM3 function	
//		printf("TIM3_IRQHandler (5ms) \r\n");
		if (GPIO_ReadInputDataBit(AudioControl_T2_GPIO_PORT ,AudioControl_T2_PIN)== TRUE)
		{
			printf("22222222222!\r\n");	
		}
		
		capture = TIM_GetCapture2(TIM3);
		TIM_SetCompare2(TIM3, capture + CCR2_Val);			
	}
}

void USART1_IRQHandler(void)
{
//	uint16_t tmp =0 ;	
	if(USART_GetITStatus(AudioControl_USART, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
//	    tmp = (USART_ReceiveData(AudioControl_UART));
//		printf("Input : %c \r\n" , tmp);
		USART_TEST();

		USART_ClearFlag(AudioControl_USART , USART_FLAG_RXNE);
	}  
}

void EXTI0_1_IRQHandler(void)
{
	if(EXTI_GetITStatus(AudioControl_T1_EXTI_LINE) != RESET)
	{
		//Insert EXTI function
		printf("EXTI0_1_IRQHandler(PB0)!\r\n");	

		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(AudioControl_T1_EXTI_LINE);
	}

	if(EXTI_GetITStatus(AudioControl_T2_EXTI_LINE) != RESET)
	{
		//Insert EXTI function	
		printf("EXTI0_1_IRQHandler(PB1)!\r\n");	

		/* Clear the EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(AudioControl_T2_EXTI_LINE);
	}
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
