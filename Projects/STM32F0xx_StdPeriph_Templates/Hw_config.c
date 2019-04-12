/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(AudioControl_USART, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(AudioControl_USART, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef 	RCC_ClockFreq;

/*ADC variable*/
uint16_t			ADCOld=0;
uint16_t			ADValue=0;
uint16_t			ADCBuffer[ADC_Number];	
uint8_t				ADCCounter=0;
__IO uint16_t 		RegularConvData_Tab[2];

/*SPI variable*/
uint8_t sDummy=0x5A;
uint8_t Rx_Buffer1[480];

/*TIMER variable*/
__IO uint16_t 		CCR1_Val = 6000;	//	6000000/6000 = 1000Hz = 1ms
__IO uint16_t 		CCR2_Val = 1200;	//	6000000/1200 = 5000Hz = 5ms

/*FLASH read/write variable*/
uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t Data = 0x3210ABCD;
uint32_t NbrOfPage = 0x00;
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO TestStatus MemoryProgramStatus = PASSED;
uint8_t TransBuffer[FLASH_USER_END_ADDR-FLASH_USER_START_ADDR];

typedef void (*f1_p)(uint32_t,uint32_t);
f1_p Flash_Erase_p = ((void (*) (uint32_t,uint32_t)) (0x08008000+1))	;
typedef void (*f2_p)(uint32_t,uint32_t,uint32_t);
f2_p Flash_Write_p = ((void (*) (uint32_t,uint32_t,uint32_t)) (0x08008100+1));
f2_p Flash_Read_p = ((void (*) (uint32_t,uint32_t,uint32_t)) (0x08008200+1));

/*
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to APB1 clock (PCLK1),  
      => TIM3CLK = PCLK1 = SystemCoreClock = 48 MHz
          
    To get TIM3 counter clock at 6 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = (PCLK1 /6 MHz) - 1
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = 1000 Hz
    CC2 update rate = TIM3 counter clock / CCR2_Val = 5000 Hz    
*/
void TIM_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	uint16_t PrescalerValue = 0;
	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	PrescalerValue = (uint16_t) ((SystemCoreClock) / 6000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
	
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void EXTI_Config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(AudioControl_T1_GPIO_CLK|AudioControl_T2_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = AudioControl_T1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(AudioControl_T1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AudioControl_T2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(AudioControl_T2_GPIO_PORT, &GPIO_InitStructure);	

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(AudioControl_T1_EXTI_PORT_SOURCE, AudioControl_T1_EXTI_PIN_SOURCE);
	SYSCFG_EXTILineConfig(AudioControl_T2_EXTI_PORT_SOURCE, AudioControl_T2_EXTI_PIN_SOURCE);
	
	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = AudioControl_T1_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = AudioControl_T2_EXTI_LINE;
	EXTI_Init(&EXTI_InitStructure);	

	/* Enable and set EXTI0_1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = AudioControl_T1_EXTI_IRQn|AudioControl_T1_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/* Private functions ---------------------------------------------------------*/

void ADC_Average(void)
{
	uint32_t	ValueX=0;
	uint8_t 	i=0;

	if(ADCCounter<ADC_Number)
	{
		ADCBuffer[ADCCounter]=ADCOld;
		ADCCounter++;
	}
	else	//when counter > ADC_Number , reset counter
	{
		ADCCounter = 0;
	}

	for(i=0;i<ADC_Number;i++)
	{
		ValueX+=ADCBuffer[i];
	}
	
	ValueX/=ADC_Number;
	ADValue=(uint16_t)ValueX;
//	ADVoltage=((uint16_t)ValueX*ADC_VERF_MV)/0xFFF;

}	

void ADC_ConversionOutput(void)
{
	uint16_t ADC1ConvertedValue =0;
	uint16_t ADC1ConvertedVoltage=0;
	uint16_t ADCNew = 0 ;
//	uint32_t i=0;

    /* Test DMA1 TC flag */
    while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ); 
    
    /* Clear DMA TC flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
    
	ADCNew = RegularConvData_Tab[1];		
	ADCOld=(ADCNew+ADCOld)/2;
	ADC_Average();

//	for (i=0;i<2;i++)
//		{
//			printf("Value %d= %4d \r\n" ,i, RegularConvData_Tab[i]);
//		}

//	printf("Value 0= %2d \r\n" , RegularConvData_Tab[0]);
//	printf("Value 1= %2d \r\n" , RegularConvData_Tab[1]);
	
	ADC1ConvertedValue = ADValue;
//	printf("ADC1ConvertedValue = %d \r\n" , ADC1ConvertedValue);
    
    /* Compute the voltage */
    ADC1ConvertedVoltage = (ADC1ConvertedValue *AudioControl_ADC1_VERF_MV)/0xFFF;
 	printf("ADC1ConvertedVoltage = %d (mv)\r\n" , ADC1ConvertedVoltage);   

}

void ADC_DMA_Config(void)
{
	DMA_InitTypeDef   	DMA_InitStructure;
	ADC_InitTypeDef     ADC_InitStructure;
	GPIO_InitTypeDef    GPIO_InitStructure;
	
	/* ADC1 DeInit */  
	ADC_DeInit(AudioControl_ADC1);

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(AudioControl_ADC1_TEMP_1_GPIO_CLK, ENABLE);

	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(AudioControl_ADC1_TEMP_1_CLK, ENABLE);

	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

	/* Configure ADC Channel11 and channel10 as analog input */

	GPIO_InitStructure.GPIO_Pin = AudioControl_ADC1_TEMP_1_PIN|AudioControl_ADC1_TEMP_2_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_Init(AudioControl_ADC1_TEMP_1_GPIO_PORT, &GPIO_InitStructure);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR ;//ADC1_DR_Address , 0x40012440
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);
  
	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* Configure the ADC1 in continuous mode withe a resolution equal to 12 bits  */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;	
	ADC_Init(AudioControl_ADC1, &ADC_InitStructure); 

	/* Convert the ADC1 Channel11 and channel10 with 55.5 Cycles as sampling time */ 
	ADC_ChannelConfig(AudioControl_ADC1, ADC_Channel_1 , ADC_SampleTime_239_5Cycles); 
	ADC_ChannelConfig(AudioControl_ADC1, ADC_Channel_2 , ADC_SampleTime_239_5Cycles); 

	/* ADC Calibration */
	ADC_GetCalibrationFactor(AudioControl_ADC1);

	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(AudioControl_ADC1, ADC_DMAMode_Circular);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* Enable ADC_DMA */
	ADC_DMACmd(AudioControl_ADC1, ENABLE);  

	/* Enable the ADC peripheral */
	ADC_Cmd(AudioControl_ADC1, ENABLE);     

	/* Wait the ADRDY flag */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
	
	/* ADC1 regular Software Start Conv */ 
	ADC_StartOfConversion(AudioControl_ADC1);
}

/*
	[IAR stm32f030_flash.icf file add]
	place at address mem:0x08008000 { readonly section .Flash_Erase};
	place at address mem:0x08008100 { readonly section .Flash_Read};
	place at address mem:0x08008200 { readonly section .Flash_Write};


	CALL Flash_Erase function :
	((void  (void)) (0x08008000+1)) (); 

	CALL Flash_Read function : 
	((void  (void)) (0x08008100+1)) (); 

	CALL Flash_Write function : 
	((void  (void)) (0x08008200+1)) (); 

*/

void Flash_Erase(uint32_t start , uint32_t end) @".Flash_Erase"
{
	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();

	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

	/* Define the number of page to be erased */
	NbrOfPage = (end - start) / FLASH_PAGE_SIZE;

	/* Erase the FLASH pages */
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		if (FLASH_ErasePage(start + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
		{
			/* Error occurred while sector erase. 
			User can add here some code to deal with this error  */
			while (1)
			{
			}
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock(); 
	printf("Flash_Erase!\r\n");
}

void Flash_Read(uint32_t start , uint32_t end , uint32_t DataRead) @".Flash_Read"
{
	/* Check if the programmed data is OK 
	  MemoryProgramStatus = 0: data programmed correctly
	  MemoryProgramStatus != 0: number of words not programmed correctly ******/
//	uint32_t i=0,j=0;
	Address = start;
	MemoryProgramStatus = PASSED;

	while (Address < end)
	{
		Data = *(__IO uint32_t *)Address;

		if (Data != DataRead)
		{
			MemoryProgramStatus = FAILED;  
		}

		Address = Address + 4;
	}
	printf("MemoryProgramStatus = %d\r\n" ,MemoryProgramStatus );

	#if 0
	printf("Flash_Read!\r\n");	
	for (i=0,j=0;i<(end-start);i++)
	{
		printf("0x%8X,",*(__IO uint32_t *)start+i);
		j++;
		if (j>=4)
		{
			printf("\r\n");
			j=0;
		}
	}
	printf("\r\n");	
	#endif
}

void Flash_Write(uint32_t start , uint32_t end , uint32_t DataWrite) @".Flash_Write"
{
//	uint32_t i=0;
	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();

	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

	/* Program the user Flash area word by word
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	Address = start;

	#if 0	//array
	for (i=0;i<(FLASH_USER_END_ADDR-FLASH_USER_START_ADDR);i++)
	{
		TransBuffer[i] = 0x01+i;
	};
	
	i=0;
	while (Address < end)
	{
		if (FLASH_ProgramWord(Address, (uint32_t)TransBuffer[i]) == FLASH_COMPLETE)
		{
			Address = Address + 4;
		}
		else
		{ 
			/* Error occurred while writing data in Flash memory. 
			User can add here some code to deal with this error */
			while (1)
			{
			}
		}
		i++;
	}
	#else	//single
	while (Address < end)
	{
		if (FLASH_ProgramWord(Address, DataWrite) == FLASH_COMPLETE)
		{
			Address = Address + 4;
		}
		else
		{ 
			/* Error occurred while writing data in Flash memory. 
			User can add here some code to deal with this error */
			while (1)
			{
			}
		}
	}
	#endif

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock(); 
	printf("Flash_Write!\r\n");
}

void Flash_MEMTEST(uint32_t StartAddress , uint32_t Length)
{

	Flash_Erase_p(FLASH_USER_START_ADDR+0x1000,FLASH_USER_END_ADDR+0x1000);
	Flash_Write_p(FLASH_USER_START_ADDR+0x1000,FLASH_USER_END_ADDR+0x1000,DATA_32r);
	Flash_Read_p(FLASH_USER_START_ADDR+0x1000,FLASH_USER_END_ADDR+0x1000,DATA_32r);

	printf("Flash_Erase Add:0x%8X \r\n" , (uint32_t) &Flash_Erase);
	printf("Flash_Read Add:0x%8X \r\n" , (uint32_t) &Flash_Read);
	printf("Flash_Write Add:0x%8X \r\n" , (uint32_t) &Flash_Write);	

	#if 0
	uint16_t i=0;
	uint8_t j=0;
	
	memcpy(TransBuffer,(uint32_t*)&Flash_Erase,Length);

	printf("TransBuffer = \r\n");
	for (i=0,j=0;i<0xFF;i++)
	{
		printf("0x%8X ,",TransBuffer[i]);
		j++;
		if (j>=4)
		{
			printf("\r\n");
			j=0;
		}
	}
	printf("\r\n");	
	#endif

	#if 0
	uint16_t i=0;	
	Flash_Erase(StartAddress , StartAddress+Length);
	Flash_Write(StartAddress,StartAddress+4,TransBuffer[0]);
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

	NbrOfPage = ((Length+StartAddress)-StartAddress) / FLASH_PAGE_SIZE; // start 32k address 

	/* Erase the FLASH pages */
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		if (FLASH_ErasePage(StartAddress + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE)
		{
			/* Error occurred while sector erase. 
			User can add here some code to deal with this error  */
			while (1)
			{
			}
		}
	}

	Address = StartAddress;
	i= 0;
	while (Address < (Length+StartAddress))
	{
		if (FLASH_ProgramWord(Address, TransBuffer[i]) == FLASH_COMPLETE)
		{
			Address = Address + 4;
		}
		else
		{ 
			/* Error occurred while writing data in Flash memory. 
			User can add here some code to deal with this error */
			while (1)
			{
			}
		}
		i++;
	}

	FLASH_Lock(); 	
	#endif
}

void SPI_DMABufferStart(uint8_t CMD, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
	DMA_InitTypeDef DMA_InitStructure;

	// TODO:NSS set LOW
	//NCS_Low();

	while (SPI_I2S_GetFlagStatus(AudioControl_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_SendData8(AudioControl_SPI,CMD);//AudioControl_SPI_CMD01
	while (SPI_I2S_GetFlagStatus(AudioControl_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_ReceiveData8(AudioControl_SPI);

	/* DMA configuration -------------------------------------------------------*/
	/* Deinitialize DMA Streams */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

	DMA_DeInit(DMA1_Channel3);
	DMA_DeInit(DMA1_Channel2);

	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = NumByteToWrite ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* Configure TX DMA */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST ;
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t)pBuffer ;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = NumByteToWrite ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* Configure RX DMA */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC ;
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t)&sDummy ; 
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	/* Enable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Channel3,ENABLE);
	/* Enable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Channel2,ENABLE);
	/* Enable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(AudioControl_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
	/* Enable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(AudioControl_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
	/* The Data transfer is performed in the SPI using Direct Memory Access */

}

void SPI_DMABufferWait(void)
{
	/* Waiting the end of Data transfer */
	while (DMA_GetFlagStatus(DMA1_FLAG_TC3)==RESET);
	while (DMA_GetFlagStatus(DMA1_FLAG_TC2)==RESET);
	/* Clear DMA Transfer Complete Flags */
	DMA_ClearFlag(DMA1_FLAG_TC3);
	DMA_ClearFlag(DMA1_FLAG_TC2);  
	/* Disable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Channel3,DISABLE);
	/* Disable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Channel2,DISABLE);  
	/* Disable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(AudioControl_SPI, SPI_I2S_DMAReq_Tx, DISABLE);
	/* Disable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(AudioControl_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

	// TODO:NSS set HIGH
	//NCS_High();
}

void SPI_CMD_EXAMPLE(uint8_t Cmd)
{
//	__IO uint8_t temp;
//	uint32_t i=0;

	#if 1
	while (SPI_I2S_GetFlagStatus(AudioControl_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_SendData8(AudioControl_SPI,Cmd);
	while (SPI_I2S_GetFlagStatus(AudioControl_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_ReceiveData8(AudioControl_SPI);
	printf("SPI_CMD_EXAMPLE(0x%X)\r\n",Cmd);
	#else
	for(i=0;i<480;i++)
	{
		Rx_Buffer1[i]=i;
	}

	SPI_DMABufferStart(AudioControl_SPI_CMD01,Rx_Buffer1 ,480);
	SPI_DMABufferWait();
	
	for(i=0;i<240;i++)
	{
		temp = Rx_Buffer1[i*2];
		Rx_Buffer1[i*2] = Rx_Buffer1[i*2 + 1];
		Rx_Buffer1[i*2 + 1] = temp;			
	}
	
	SPI_DMABufferStart(0x72,Rx_Buffer1 ,480);
	SPI_DMABufferWait();
	#endif 	
}

void SPI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable the SPI periph */
	RCC_APB2PeriphClockCmd(AudioControl_SPI_CLK, ENABLE);

	/* Enable SCK, MOSI, MISO and NSS GPIO clocks */
	RCC_AHBPeriphClockCmd(AudioControl_SPI_GPIO_CLK, ENABLE);

	GPIO_PinAFConfig(AudioControl_SPI_CLK_PORT, 	AudioControl_SPI_CLK_SOURCE, AudioControl_SPI_CLK_AF);
	GPIO_PinAFConfig(AudioControl_SPI_MISO_PORT, AudioControl_SPI_MISO_SOURCE, AudioControl_SPI_MISO_AF);
	GPIO_PinAFConfig(AudioControl_SPI_MOSI_PORT, AudioControl_SPI_MOSI_SOURCE, AudioControl_SPI_MOSI_AF);
	GPIO_PinAFConfig(AudioControl_SPI_NSS_PORT, AudioControl_SPI_NSS_SOURCE, AudioControl_SPI_NSS_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = AudioControl_SPI_CLK_PIN;
	GPIO_Init(AudioControl_SPI_CLK_PORT, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  AudioControl_SPI_MOSI_PIN;
	GPIO_Init(AudioControl_SPI_MOSI_PORT, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = AudioControl_SPI_MISO_PIN;
	GPIO_Init(AudioControl_SPI_MISO_PORT, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = AudioControl_SPI_NSS_PIN;
	GPIO_Init(AudioControl_SPI_NSS_PORT, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(AudioControl_SPI);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(AudioControl_SPI, &SPI_InitStructure);

	/* Initialize the FIFO threshold */
	SPI_RxFIFOThresholdConfig(AudioControl_SPI, SPI_RxFIFOThreshold_QF);

	/* Enable the SPI peripheral */
	SPI_Cmd(AudioControl_SPI, ENABLE);

	/* Enable NSS output for master mode */
	SPI_SSOutputCmd(AudioControl_SPI, ENABLE);
}

void USART_TX_Polling(void)
{
	uint8_t Char = 0;
	
	/* Wait until TXE flag is set */    
	while(USART_GetFlagStatus(AudioControl_USART, USART_FLAG_TXE) == RESET)
	{}

	Char = USART_ReceiveData(AudioControl_USART) ;

	if (Char == 'x')
	{
		USART_SendData(AudioControl_USART, 'X'); 
	}

	while (USART_GetFlagStatus(AudioControl_USART, USART_FLAG_TC) == RESET)
	{}	
}

void USART_TEST(void)
{
	__IO uint8_t temp;
	
	if(USART_GetFlagStatus(AudioControl_USART, USART_FLAG_RXNE) == SET)
	{
			temp = USART_ReceiveData(AudioControl_USART);
			printf("Press KEY : %c \n\r",temp);

			switch (temp)
			{
				case '1': 
//					EXTI_GenerateSWInterrupt(EXTI_Line2);
					EXTI_GenerateSWInterrupt(EXTI_Line0);					
					printf("EXTI_GenerateSWInterrupt(EXTI_Line0) \r\n");	

					break;

				case '2': 
//					EXTI_GenerateSWInterrupt(EXTI_Line2);
					EXTI_GenerateSWInterrupt(EXTI_Line1);					
					printf("EXTI_GenerateSWInterrupt(EXTI_Line1) \r\n");	

					break;

				case '3':
					EXTI_GenerateSWInterrupt(EXTI_Line3);					
					printf("EXTI_GenerateSWInterrupt(EXTI_Line3) \r\n");					
					break;

				case '4':
					
					break;					
					
				default : 
					printf("INPUT CMD not support !\r\n");
					break;
			}
	}
}

void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Clock configuration ---------------------------------------------------*/
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(AudioControl_USART_GPIO_CLK, ENABLE);

	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(AudioControl_USART_CLK, ENABLE);

	/* GPIO configuration ----------------------------------------------------*/
	GPIO_DeInit(AudioControl_USART_GPIO_PORT);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(AudioControl_USART_GPIO_PORT, AudioControl_USART_TX_SOURCE, AudioControl_USART_TX_AF);
	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(AudioControl_USART_GPIO_PORT, AudioControl_USART_RX_SOURCE, AudioControl_USART_RX_AF);

	/* Configure USARTx_Tx,USARTx_Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = AudioControl_USART_TX_PIN|AudioControl_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(AudioControl_USART_GPIO_PORT, &GPIO_InitStructure);

	/* USART configuration ---------------------------------------------------*/
	USART_DeInit(AudioControl_USART);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(AudioControl_USART, &USART_InitStructure);

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(AudioControl_USART, USART_IT_RXNE, ENABLE); 
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	/* The software must wait until TC=1. The TC flag remains cleared during all data
	transfers and it is set by hardware at the last frame’s end of transmission*/	
	while (USART_GetFlagStatus(AudioControl_USART, USART_FLAG_TC) == RESET)
	{}

	/* NVIC configuration */
	/* Enable the USARRx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = AudioControl_USART_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	/* Enable the USARRx */
	USART_Cmd(AudioControl_USART, ENABLE);
}

void SysTickTimer_Config(void)
{
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	#if 1 //debug
	printf("===========================\r\n");
	printf("SYSCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.SYSCLK_Frequency);
	printf("HCLK_Frequency = %d \r\n" , 			RCC_ClockFreq.HCLK_Frequency);
	printf("PCLK_Frequency = %d \r\n" , 			RCC_ClockFreq.PCLK_Frequency);
	printf("ADCCLK_Frequency= %d \r\n" , 		RCC_ClockFreq.ADCCLK_Frequency);
	printf("CECCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.CECCLK_Frequency);
	printf("I2C1CLK_Frequency = %d \r\n" , 		RCC_ClockFreq.I2C1CLK_Frequency);
	printf("USART1CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.USART1CLK_Frequency); 
	#endif /*debug*/
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x01);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay_ms(__IO uint32_t uTime)
{ 
	uwTimingDelay = uTime;
	while(uwTimingDelay != 0);
}

void Delay_s(__IO uint32_t mTime)
{ 
	uint32_t i;
	for(i=0;i<mTime;i++)
		Delay_ms(1000);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

//currently not use
/*

void SystemClkDelay(void)
{
	uint32_t i;
	i = 0xffff;
	while(i--);
}

void wtPutChar(uint8_t ccc)
{
	UART1_SendData8(ccc);
	// Loop until the end of transmission 
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	
}

u16 GetAbsTime(u16 a,u16 b)
{
	u16 c;
	if(a>=b) c=(a-b);
	else c=65535-(b-a);	
	
	return c;
}
*/

void UART_SendByte(uint8_t Data)
{
	USART_SendData(AudioControl_USART , (unsigned char)Data);
	while (USART_GetFlagStatus(AudioControl_USART , USART_FLAG_TXE)==RESET);
	{
	}
}

void UART_SendString(uint8_t* Data,uint16_t len)
{
	uint16_t i=0;
	for(i=0;i<len;i++ )
	{
		UART_SendByte(Data[i]);
	}
}

void SystemClkDelay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}


