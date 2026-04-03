Title:
STM32 Peripherial ADC driver

Decription:
Directory contains source and header files, which were build to help others in working with ADC.

    Main purpose of ADC driver (STM32 peripherial) is to store functions and macros whiich will simplify workflow with ADC. Last but not least purpose is to make all firmwares, written in PCBs' cores of Perła, universal. This will help in locating, analyzing and debuging errors in software.


    NOTE:
    Drivers in being built using registers and low-level features of ADC controller embedded in STM32 cores. Driver is universal for almost all type of STM32 cores

Application:
To ensure that code won't blow HardFault, please call Init function when working with ADC. Note that, when dual mode is need, call Init_Multimode function in Init function of ADC Slave (MX_ADC_Init)

Files listing: 1. Inc/adc_driver.h - function prototypes, macros, structs 2. Inc/stm32_family.h - macros of stm32 families definition 3. Src/adc_driver.c - functions' bodies, variables' definitions

Status:
General:
Driver build is done. Code tests are done.
Driver supports functionalities like: - ADC Non-Continous Conversion - ADC Continous Conversion - ADC with DMA Normal mode - ADC with DMA Circular mode - ADC Multimode - Auto detect number of channels used for data conversion

    Detailed:
        1. Inc/adc_driver.h   [DONE]
        2. Inc/stm32_family.h [DONE]
        3. Src/adc_driver.c   [DONE]

Manual: 
    
    1. For proper workflow, for each ADC initialize ADC_BufferTypeDef and ADC_ChannelsTypeDef. If two ADCs are in dual mode, then only one ADC_BufferTypeDef object is needed. 
    2. For proper initialization, call function: ADC_Init in each MX_ADC_Iniit function for all ADC You're working with. 
    3. If multimode is needed, then call function ADC_InitMultimode

Author:
Bartosz Rychlicki
bartoszrychl@student.agh.edu.pl

    AGH Eko-Energy
    Software department
