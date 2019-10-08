/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f4xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * @brief timer instances
 */
extern TIM_HandleTypeDef tim_chkup;            // PID timer

/*
 * @brief motor steps.
 */
extern int32_t dir_motor1, dir_motor2;
extern int32_t steps_motor1, steps_motor2;

/*
 * @brief motors checkup parameters
 */
extern uint32_t chkup_pending;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/
/*
 * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

/*
 * @name   TIM3_IRQHandler.
 * @brief  handles timer3 period elapsed callback IRQ.
 * @param  none.
 * @retval none.
 */
void
TIM3_IRQHandler (void)
{
  HAL_TIM_IRQHandler (&tim_chkup);

  /* rise update flag */
  chkup_pending = 1;
//  do_motor_correction ();
}

/*
 * @name   EXTI4_IRQHandler.
 * @brief  handles encoder 1 IRQ.
 * @param  none.
 * @retval none.
 */
void
EXTI4_IRQHandler (void)
{
  HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_4);

  steps_motor1 += dir_motor1;
}

/*
 * @name   EXTI9_5_IRQHandler.
 * @brief  handles encoder 2 IRQ.
 * @param  none.
 * @retval none.
 */
void
EXTI9_5_IRQHandler (void)
{
  HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_5);
  HAL_NVIC_ClearPendingIRQ (EXTI9_5_IRQn);

  steps_motor2 += dir_motor2;
}
