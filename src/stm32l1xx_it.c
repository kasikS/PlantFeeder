/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Template/stm32l1xx_it.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    May-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"
#include "discover_board.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_rtc.h"
#include "stm32l_discovery_lcd.h"
#include "tsl.h"
#include "delay.h"
#include "serial.h"
#include "ADC.h"
#include "timer.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void) {}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
        ;
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
        ;
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
        ;
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
        ;
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
        ;
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
        ;
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
        ;
}

/**
  * @brief  This function handles SysTick interrupts.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void) {
    /*TSL_tim_ProcessIT();*/
    TimingDelay_Decrement();
}


void EXTI0_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line0);
}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_md.s).                                            */
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

#define MAX_STRLEN 12
volatile char received_string[MAX_STRLEN+1];


void USART1_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){

		static unsigned int cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t

		/* check if the received character is not the LF character (used to determine end of string)
		 * or the if the maximum string length has been been reached
		 */
		if( (t != '\r') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			UART_puts(USART1, received_string);
		}

	}
}

void USART2_IRQHandler(void){

	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){

		static unsigned int cnt = 0; // this counter is used to determine the string length
		char t = USART2->DR; // the character from the USART2 data register is saved in t

		/* check if the received character is not the LF character (used to determine end of string)
		 * or the if the maximum string length has been been reached
		 */
		if( (t != '\r') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			UART_puts(USART2, received_string);
		}

	}
}



void ADC1_IRQHandler(void)
{
	ADC1_SetRdy();
 // if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) //
 // {
    /* Clear ADC1 EOC pending interrupt bit */
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

  //}
}

void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        Timer3_SetRdy();
//        UART_puts(USART1, "b \r\n");
    }
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
