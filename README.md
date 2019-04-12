# STM32F0xx_StdPeriph_Lib_V1.3.1_IARFunctionAllocate

update @ 2019/04/12

Add function allocate example in IAR

function with specific function point and define section in ICF file , will allocate to specific address

1. check Flash_Erase_p , Flash_Write_p , Flash_Read_p in Hw_config.c for example. 

2. function Flash_Erase , Flash_Read , Flash_Write , allocate in specific flash address

3. check readonly section in stm32f030_flash.icf for example