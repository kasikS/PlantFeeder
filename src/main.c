#include "main.h"
#include "delay.h"

#define enableGlobalInterrupts()   __set_PRIMASK(0);
#define disableGlobalInterrupts()  __set_PRIMASK(1);

void RCC_Configuration(void);
void Init_GPIOs (void);

int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
        this is done through SystemInit() function which is called from startup
        file (startup_stm32l1xx_md.s) before to branch to application main.
        To reconfigure the default setting of SystemInit() function, refer to
        system_stm32l1xx.c file
        */

    /* Configure Clocks for Application need */
    RCC_Configuration();

    /* Set internal voltage regulator to 1.8V */
    PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);

    /* Wait Until the Voltage Regulator is ready */
    while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;

    /* Init I/O ports */
    Init_GPIOs();

    /* Enable General interrupts */
    enableGlobalInterrupts();

    /* Init Touch Sensing configuration */
    /*TSL_user_Init();*/

    DelayInit();

    /* Initializes the LCD glass */
    LCD_GLASS_Init();
    LCD_GLASS_DisplayString((unsigned char*)"Hello");

    /* Switch off the leds*/
    GPIO_HIGH(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);
    GPIO_HIGH(LD_GPIO_PORT,LD_BLUE_GPIO_PIN);

    /*Until application reset*/
    while (1)
    {
        GPIO_TOGGLE(LD_GPIO_PORT,LD_BLUE_GPIO_PIN);
        GPIO_TOGGLE(LD_GPIO_PORT,LD_GREEN_GPIO_PIN);
        Delay(1000);
    }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
    /* Enable HSI Clock */
    RCC_HSICmd(ENABLE);

    /*!< Wait till HSI is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

    /* Set HSI as sys clock*/
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

    /* Set MSI clock range to ~4.194MHz*/
    RCC_MSIRangeConfig(RCC_MSIRange_6);

    /* Enable the GPIOs clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                              RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                              RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH,
                          ENABLE);

    /* Enable comparator, LCD and PWR mngt clocks */
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_COMP | RCC_APB1Periph_LCD | RCC_APB1Periph_PWR, ENABLE);

    /* Enable ADC & SYSCFG clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG, ENABLE);

    /* Allow access to the RTC */
    PWR_RTCAccessCmd(ENABLE);

    /* Reset RTC Backup Domain */
    RCC_RTCResetCmd(ENABLE);
    RCC_RTCResetCmd(DISABLE);

    /* LSE Enable */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait until LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
        ;

    /* RTC Clock Source Selection */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable the RTC */
    RCC_RTCCLKCmd(ENABLE);

    /*Disable HSE*/
    RCC_HSEConfig(RCC_HSE_OFF);
    if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET) {
        /* Stay in infinite loop if HSE is not disabled*/
        while (1) ;
    }
}

/**
  * @brief  To initialize the I/O ports
  * @caller main
  * @param None
  * @retval None
  */
void  Init_GPIOs (void)
{
    /* GPIO, EXTI and NVIC Init structure declaration */
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure User Button pin as input */
    GPIO_InitStructure.GPIO_Pin = USERBUTTON_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(USERBUTTON_GPIO_PORT, &GPIO_InitStructure);

    /* Select User Button pin as input source for EXTI Line */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* Configure EXT1 Line 0 in interrupt mode trigged on Rising edge */
    EXTI_InitStructure.EXTI_Line =
        EXTI_Line0;  // PA0 for User button AND IDD_WakeUP
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI0 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure the LED_pin as output push-pull for LD3 & LD4 usage*/
    GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);

    /* Force a low level on LEDs*/
    GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
    GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);

    /* Counter enable: GPIO set in output for enable the counter */
    GPIO_InitStructure.GPIO_Pin = CTN_CNTEN_GPIO_PIN;
    GPIO_Init(CTN_GPIO_PORT, &GPIO_InitStructure);

    /* To prepare to start counter */
    GPIO_HIGH(CTN_GPIO_PORT, CTN_CNTEN_GPIO_PIN);

    /* Configure Port A LCD Output pins as alternate function */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                                  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Select LCD alternate function for Port A LCD Output pins */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_LCD);

    /* Configure Port B LCD Output pins as alternate function */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 |
                                  GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Select LCD alternate function for Port B LCD Output pins */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_LCD);

    /* Configure Port C LCD Output pins as alternate function */
    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 |
        GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Select LCD alternate function for Port B LCD Output pins */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource0, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_LCD);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_LCD);
}


/**
  * @brief  Executed when a sensor is in Error state
  * @param  None
  * @retval None
  */
void MyLinRots_ErrorStateProcess(void)
{
    // Add here your own processing when a sensor is in Error state
    TSL_linrot_SetStateOff();
}


/**
  * @brief  Executed when a sensor is in Off state
  * @param  None
  * @retval None
  */
void MyLinRots_OffStateProcess(void)
{
    // Add here your own processing when a sensor is in Off state
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
  while (1);
}

#endif
