/*
 ******************************************************************************
 * @file    main.c
 * @module  app
 * @author  Mohamed Hnezli
 * @version V0.2
 * @date    27-09-2019
 * @brief   implements basic robot driver.
 *
 ******************************************************************************
 * @attention
 *
 * - The present software had been tested only on STM32F3CCTx and may be non
 * functional on other targets.
 *
 * <h2><center>&copy COPYRIGHT 2019 Mohamed Hnezli </center></h2>
 ******************************************************************************
 */

/* include ------------------------------------------------------------------ */
#include "stm32f4xx_hal.h"             // MCU HAL
#include "stm32f4_discovery.h"         // BSP
#include <string.h>                    // strlen


/* define ------------------------------------------------------------------- */
/*
 * @brief samplers GPIO ports and pins
 */
#define GPIO_PORT_SPEED_MOTOR1         GPIOD
#define GPIO_PIN_SPEED_MOTOR1          GPIO_PIN_14
#define GPIO_AF_SPEED_MOTOR1           GPIO_AF2_TIM4
#define GPIO_CLK_EN_SPEED_MOTOR1()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PORT_SPEED_MOTOR2         GPIOD
#define GPIO_PIN_SPEED_MOTOR2          GPIO_PIN_15
#define GPIO_AF_SPEED_MOTOR2           GPIO_AF2_TIM4
#define GPIO_CLK_EN_SPEED_MOTOR2()     __HAL_RCC_GPIOD_CLK_ENABLE()

#define GPIO_PORT_DIR_MOTOR1_1         GPIOD
#define GPIO_PIN_DIR_MOTOR1_1          GPIO_PIN_10
#define GPIO_CLK_EN_DIR_MOTOR1_1()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PORT_DIR_MOTOR1_2         GPIOD
#define GPIO_PIN_DIR_MOTOR1_2          GPIO_PIN_11
#define GPIO_CLK_EN_DIR_MOTOR1_2()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PORT_DIR_MOTOR2_1         GPIOD
#define GPIO_PIN_DIR_MOTOR2_1          GPIO_PIN_12
#define GPIO_CLK_EN_DIR_MOTOR2_1()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PORT_DIR_MOTOR2_2         GPIOD
#define GPIO_PIN_DIR_MOTOR2_2          GPIO_PIN_13
#define GPIO_CLK_EN_DIR_MOTOR2_2()     __HAL_RCC_GPIOD_CLK_ENABLE()

#define GPIO_PORT_ENCODER1             GPIOB
#define GPIO_PIN_ENCODER1              GPIO_PIN_4  //PA0
#define GPIO_AF_ENCODER1               GPIO_AF2_TIM3//GPIO_AF2_TIM5
#define GPIO_CLK_EN_ENCODER1()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPIO_PORT_ENCODER2             GPIOB
#define GPIO_PIN_ENCODER2              GPIO_PIN_5
#define GPIO_AF_ENCODER2               GPIO_AF2_TIM3
#define GPIO_CLK_EN_ENCODER2()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define GPIO_PORT_SERIAL_TX            GPIOC
#define GPIO_PIN_SERIAL_TX             GPIO_PIN_10
#define GPIO_CLK_EN_SERIAL_TX()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define GPIO_PORT_SERIAL_RX            GPIOC
#define GPIO_PIN_SERIAL_RX             GPIO_PIN_11
#define GPIO_CLK_EN_SERIAL_RX()        __HAL_RCC_GPIOC_CLK_ENABLE()

/*
 * @brief peripheral interface
 */
#define TIMER_IFACE_MOTORS             TIM4
#define TIMER_CH_MOTOR1                TIM_CHANNEL_3
#define TIMER_CH_MOTOR2                TIM_CHANNEL_4
#define TIMER_CLK_EN_MOTORS()          __TIM4_CLK_ENABLE()
#define TIMER_IFACE_CHKUP              TIM3
#define TIMER_CLK_EN_CHKUP()           __TIM3_CLK_ENABLE()

#define UART_SERIAL                    UART4
#define UART_CLK_EN_SERIAL()           __HAL_RCC_UART4_CLK_ENABLE()

/*
 * @brief IRQ configuration
 */
#define IRQ_LINE_TIMER_CHKUP           TIM3_IRQn
#define IRQ_PRIO_TIMER_CHKUP           2
#define IRQ_LINE_ENCODER1              EXTI4_IRQn
#define IRQ_PRIO_ENCODER1              3
#define IRQ_LINE_ENCODER2              EXTI9_5_IRQn
#define IRQ_PRIO_ENCODER2              3

/*
 * @brief serial line parameters.
 */
#define SERIAL_BAUDRATE                115200

/*
 * @brief motors abstraction parameters
 */
#define MOTOR_IDX_1                    1
#define MOTOR_IDX_2                    2

#define MOTOR_DIR_FWD                  GPIO_PIN_SET
#define MOTOR_DIR_BWD                  GPIO_PIN_RESET

#define MOTOR_MAX_SPEED                50

#define CHECKUP_PEDIOD                 100   // [ms] TODO change me according to needs

/*
 * @define mechanical parameters
 */
#define ENCODER_PULSES_PER_SPIN        4741
#define WHEEL_PERIMETER                ((float)18.84)  // [cm]
#define MAX_TOLERATED_DIFF             100            // encoder steps


/* typedef ------------------------------------------------------------------ */
/* variable ----------------------------------------------------------------- */
/*
 * @brief timer instances
 */
TIM_HandleTypeDef tim_motors_;          // motors speed controller
TIM_HandleTypeDef tim_chkup;            // PID timer

/*
 * @brief serial interface.
 */
static UART_HandleTypeDef serial_uart_;

/*
 * @brief motor status parameter.
 */
int32_t dir_motor1 = 0, dir_motor2 = 0;
int32_t steps_motor1 = 0, steps_motor2 = 0;

/*
 * @brief motors checkup parameters
 */
uint32_t chkup_pending = 0;
uint32_t chk_timestamp = 0;

/* class -------------------------------------------------------------------- */
/* method ------------------------------------------------------------------- */
/* function ----------------------------------------------------------------- */
/*
 * @name   error_critical.
 * @brief  halts MCU on hard error occurrence.
 * @param  file: name of file in which error occurred.
 *         line: number of line in which error occurred.
 * @retval none.
 */
static void
error_critical (char* file, int line)
{
  UNUSED (file);
  UNUSED (line);

  while (1)
    {
      ; // halt program
    }

  return;
}

/*
 * @name   sysclk_Config.
 * @brief  system Clock configuration.
 * @param  none.
 * @retval none.
 * @note   HCLK is set to 24MHz, APB1 24MHz, APB2 24MHz
 */
static void
sysclk_Config (void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* initializes the CPU, AHB and APB busses clocks
   * SySclk = (((SRC / M) * N) / P) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  /* initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  return;
}

/*
 * @name   motors_init.
 * @brief  configures output signal for motors.
 * @param  none.
 * @retval none.
 * @note   timer is set to 50ms resolution.
 */
static void
motors_init (void)
{
  GPIO_InitTypeDef gpio;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  /* configure GPIO */
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_LOW;
  gpio.Pull = GPIO_NOPULL;
  GPIO_CLK_EN_DIR_MOTOR1_1 ();
  gpio.Pin = GPIO_PIN_DIR_MOTOR1_1;
  HAL_GPIO_Init(GPIO_PORT_DIR_MOTOR1_1, &gpio);
  GPIO_CLK_EN_DIR_MOTOR1_2 ();
  gpio.Pin = GPIO_PIN_DIR_MOTOR1_2;
  HAL_GPIO_Init(GPIO_PORT_DIR_MOTOR1_2, &gpio);
  GPIO_CLK_EN_DIR_MOTOR2_1 ();
  gpio.Pin = GPIO_PIN_DIR_MOTOR2_1;
  HAL_GPIO_Init(GPIO_PORT_DIR_MOTOR2_1, &gpio);
  GPIO_CLK_EN_DIR_MOTOR2_2 ();
  gpio.Pin = GPIO_PIN_DIR_MOTOR2_2;
  HAL_GPIO_Init(GPIO_PORT_DIR_MOTOR2_2, &gpio);

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_LOW;
  gpio.Pull = GPIO_NOPULL;
  GPIO_CLK_EN_SPEED_MOTOR1 ();
  gpio.Pin = GPIO_PIN_SPEED_MOTOR1;
  gpio.Alternate = GPIO_AF_SPEED_MOTOR1;
  HAL_GPIO_Init(GPIO_PORT_SPEED_MOTOR1, &gpio);
  GPIO_CLK_EN_SPEED_MOTOR2 ();
  gpio.Pin = GPIO_PIN_SPEED_MOTOR2;
  gpio.Alternate = GPIO_AF_SPEED_MOTOR2;
  HAL_GPIO_Init(GPIO_PORT_SPEED_MOTOR2, &gpio);

  /* configure timer
   * 48MHz / ((4799 + 1) * (49 + 1))
   * = 48000000 / (4700 * 50)
   * = 200Hz
   */
  TIMER_CLK_EN_MOTORS();
  tim_motors_.Instance = TIMER_IFACE_MOTORS;
  tim_motors_.Init.Prescaler = 4799;
  tim_motors_.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim_motors_.Init.Period = 49;
  tim_motors_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init (&tim_motors_);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization (&tim_motors_, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel (&tim_motors_, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel (&tim_motors_, &sConfigOC, TIM_CHANNEL_2);

  return;
}

/*
 * @name   chkup_timer_init.
 * @brief  configures timer to launches periodic check-up for motors.
 * @param  none.
 * @retval none.
 * @note   timer is set to 100ms resolution.
 */
static void
chkup_timer_init (void)
{
  /* configure timer
   * 48MHz / ((4799 + 1) * (9999 + 1))
   * = 48000000 / (4700 * 10000)
   * = 1Hz
   */
  TIMER_CLK_EN_CHKUP();
  tim_chkup.Instance = TIMER_IFACE_CHKUP;
  tim_chkup.Init.Prescaler = 4799;
  tim_chkup.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim_chkup.Init.Period = 9999;
  tim_chkup.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init (&tim_chkup);

  /* enable IRQ */
  HAL_NVIC_EnableIRQ (IRQ_LINE_TIMER_CHKUP);
  HAL_NVIC_SetPriority (IRQ_LINE_TIMER_CHKUP, IRQ_PRIO_TIMER_CHKUP, 0);

  /* launch timer */
  HAL_TIM_Base_Start_IT (&tim_chkup);

  return;
}

/*
 * @name   motor_setdir.
 * @brief  updates motors spin direction.
 * @param  motor: motor index @ref MOTOR_IDX_.
 *         dir: forward or backward @ref MOTOR_DIR_.
 * @retval none.
 * @note:  spinning forward is assumed to drive current to pin 1.
 *         reverse spinning is assumed to drive current to pin 2.
 */
static void
motor_setdir (uint32_t motor, uint32_t dir)
{
  int32_t step_val = 0;
  uint32_t dir1 = GPIO_PIN_RESET, dir2 = GPIO_PIN_RESET;

  switch (dir)
  {
  case MOTOR_DIR_FWD:
    step_val = 1;
    dir1 = GPIO_PIN_SET;
    dir2 = GPIO_PIN_RESET;
    break;
  case MOTOR_DIR_BWD:
    step_val = (-1);
    dir1 = GPIO_PIN_RESET;
    dir2 = GPIO_PIN_SET;
    break;
  default:
    step_val = 1;
    break;
  }

  switch (motor)
  {
  case MOTOR_IDX_1:
    HAL_GPIO_WritePin (GPIO_PORT_SPEED_MOTOR1, GPIO_PIN_SPEED_MOTOR1, dir);
    HAL_GPIO_WritePin (GPIO_PORT_DIR_MOTOR1_1, GPIO_PIN_DIR_MOTOR1_1, dir1);
    HAL_GPIO_WritePin (GPIO_PORT_DIR_MOTOR1_2, GPIO_PIN_DIR_MOTOR1_2, dir2);
    dir_motor1 = step_val;
    break;

  case MOTOR_IDX_2:
    HAL_GPIO_WritePin (GPIO_PORT_SPEED_MOTOR2, GPIO_PIN_SPEED_MOTOR2, dir);
    HAL_GPIO_WritePin (GPIO_PORT_DIR_MOTOR2_1, GPIO_PIN_DIR_MOTOR2_1, dir1);
    HAL_GPIO_WritePin (GPIO_PORT_DIR_MOTOR2_2, GPIO_PIN_DIR_MOTOR2_2, dir2);
    dir_motor2 = step_val;
    break;

  default:
    HAL_GPIO_WritePin (GPIO_PORT_SPEED_MOTOR1, GPIO_PIN_SPEED_MOTOR1, dir);
    HAL_GPIO_WritePin (GPIO_PORT_DIR_MOTOR1_1, GPIO_PIN_DIR_MOTOR1_1, dir1);
    HAL_GPIO_WritePin (GPIO_PORT_DIR_MOTOR1_2, GPIO_PIN_DIR_MOTOR1_2, dir2);
    dir_motor1 = step_val;
    break;
  }

  return;
}

/*
 * @name   motors_go.
 * @brief  updates motors speed.
 * @param  motor: motor index @ref MOTOR_IDX_.
 *         pulse: speed [0..MOTOR_MAX_SPEED].
 * @retval none.
 * @note   timer is set to 50ms resolution.
 */
static void
motor_go (uint32_t motor, uint16_t speed)
{
  TIM_OC_InitTypeDef sConfigOC;
  uint32_t channel;

  switch (motor)
  {
  case MOTOR_IDX_1:
    channel = TIMER_CH_MOTOR1;
    break;

  case MOTOR_IDX_2:
    channel = TIMER_CH_MOTOR2;
    break;

  default:
    channel = TIMER_CH_MOTOR1;
    break;
  }

  /* stop generation of pwm */
  HAL_TIM_PWM_Stop(&tim_motors_, channel);

  /* set new pulse width */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = speed % (MOTOR_MAX_SPEED + 1);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&tim_motors_, &sConfigOC, channel);
  HAL_TIM_PWM_Start(&tim_motors_, channel);

  return;
}

/*
 * @name   encoders_init.
 * @brief  configure encoders GPIO pins and enables IRQs.
 * @param  none.
 * @retval none.
 */
static void
encoders_init (void)
{
  GPIO_InitTypeDef gpio;
  TIM_Encoder_InitTypeDef encoderConfig;

  /* configure GPIO */
  gpio.Mode = GPIO_MODE_IT_RISING;//GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_HIGH;
  GPIO_CLK_EN_ENCODER1();
  gpio.Pin = GPIO_PIN_ENCODER1;
//  gpio.Alternate = GPIO_AF_ENCODER1;
  HAL_GPIO_Init(GPIO_PORT_ENCODER1, &gpio);
  GPIO_CLK_EN_ENCODER2();
  gpio.Pin = GPIO_PIN_ENCODER2;
//  gpio.Alternate = GPIO_AF_ENCODER2;
  HAL_GPIO_Init(GPIO_PORT_ENCODER2, &gpio);

  /* enable pins IRQs */
  HAL_NVIC_EnableIRQ (IRQ_LINE_ENCODER1);
  HAL_NVIC_SetPriority (IRQ_LINE_ENCODER1, IRQ_PRIO_ENCODER1, 0);
  HAL_NVIC_EnableIRQ (IRQ_LINE_ENCODER2);
  HAL_NVIC_SetPriority (IRQ_LINE_ENCODER2, IRQ_PRIO_ENCODER2, 1);

  return;
}

/*
 * @name   serial_init.
 * @brief  configures GPIO and UART ports of serial interface.
 * @param  none.
 * @retval none.
 */
static void
serial_init (void)
{
  GPIO_InitTypeDef gpio;
  HAL_StatusTypeDef ret_val = HAL_OK;

  /* setup GPIO pins */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF8_UART4;
  gpio.Pin = GPIO_PIN_SERIAL_TX | GPIO_PIN_SERIAL_RX;

  GPIO_CLK_EN_SERIAL_TX();
  HAL_GPIO_Init (GPIO_PORT_SERIAL_TX, &gpio);

  /* set-up hardware */
  serial_uart_.Instance = UART_SERIAL;
  HAL_UART_DeInit (&serial_uart_);
  serial_uart_.Init.BaudRate = SERIAL_BAUDRATE;
  serial_uart_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  serial_uart_.Init.Mode = UART_MODE_TX_RX;
  serial_uart_.Init.Parity = UART_PARITY_NONE;
  serial_uart_.Init.StopBits = UART_STOPBITS_1;
  serial_uart_.Init.WordLength = UART_WORDLENGTH_8B;
  serial_uart_.Init.OverSampling = UART_OVERSAMPLING_16;

  UART_CLK_EN_SERIAL();
  ret_val = HAL_UART_Init (&serial_uart_);

  if (ret_val != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  return;
}

/*
 * @name   serial_write.
 * @brief  write a string to serial interface.
 * @param  data: msg to send.
 *         len: data lenght in bytes.
 * @retval int32_t: zero: Ok, bytes transferred.
 *                  negative: transmission has failed.
 */
static int32_t
serial_write (uint8_t* data, uint32_t len)
{
  int32_t rx_status = HAL_OK;
  int32_t retval = 0;

  rx_status = HAL_UART_Transmit (&serial_uart_, (uint8_t*)data, len, 0xFFFFF);
  if (rx_status != HAL_OK)
    {
      retval = (-1);
    }

  return retval;
}

/*
 * @name   serial_read.
 * @brief  read one character from serial interface.
 * @param  char: location where to save the received character.
 * @retval int32_t: zero: Ok, one byte transferred.
 *                  negative: reception has failed.
 */
static int32_t
serial_read (const char* data)
{
  int32_t rx_status = HAL_OK;
  int32_t retval = 0;

  rx_status = HAL_UART_Transmit (&serial_uart_, (uint8_t*)data, 1, 0xFFF);
  if (rx_status != HAL_OK)
    {
      retval = (-1);
    }

  return retval;
}

/*
 * @name   do_motor_correction.
 * @brief  corrects position motors.
 * @param  none.
 * @retval none.
 */
void
do_motor_correction (void)
{
  uint32_t speed;

  /* check seconds motor */
  if (steps_motor1 - steps_motor2 > MAX_TOLERATED_DIFF)
    {
      /* do correction */
      speed = MOTOR_MAX_SPEED * (((float)(steps_motor1 - steps_motor2)) / MAX_TOLERATED_DIFF);
      if (speed > 50)
        {
          speed = 50;
        }
      else if (speed < 30)
        {
          speed = 30;
        }
      motor_go (MOTOR_IDX_2, speed);
      while (steps_motor1 - steps_motor2 > (MAX_TOLERATED_DIFF / 5 ))
        {
          ; // wait until both motors have same position
        }

      /* stop immediately */
      motor_go (MOTOR_IDX_2, 0);
      HAL_Delay (50);
    }

  /* check seconds motor */
  if (steps_motor2 - steps_motor1 > MAX_TOLERATED_DIFF)
    {
      /* do correction */
      speed = MOTOR_MAX_SPEED * (((float)(steps_motor2 - steps_motor1)) / MAX_TOLERATED_DIFF);
      if (speed > 50)
        {
          speed = 50;
        }
      else if (speed < 30)
        {
          speed = 30;
        }
      motor_go (MOTOR_IDX_1, speed);
      while (steps_motor2 - steps_motor1 > (MAX_TOLERATED_DIFF / 5))
        {
          ; // wait until both motors have same position
        }
      /* stop immediately */
      motor_go (MOTOR_IDX_1, 0);
      HAL_Delay (50);
    }

  return;
}

/*
 * @name   main.
 * @brief  motor driver implementation.
 * @param  none.
 * @retval none.
 */
int
main (void)
{
  int32_t distance1 = 0, distance2 = 0;//  [cm]

  /* configure clocks */
  sysclk_Config ();

  /* configure interfaces */
  motors_init ();
  motor_setdir (MOTOR_IDX_1, MOTOR_DIR_FWD);
  motor_setdir (MOTOR_IDX_2, MOTOR_DIR_FWD);
  encoders_init ();
//  serial_init ();
  chkup_timer_init ();

  while (1)
    {
      /* test motors */
#if 1
      /* drive motors */
      motor_setdir (MOTOR_IDX_2, MOTOR_DIR_FWD);
      motor_go (MOTOR_IDX_2, 50);
      motor_setdir (MOTOR_IDX_1, MOTOR_DIR_FWD);
      motor_go (MOTOR_IDX_1, 50);
      HAL_Delay (200);

      /* stop motors */
      motor_go (MOTOR_IDX_1, 0);
      motor_go (MOTOR_IDX_2, 0);
      HAL_Delay (100);

      /* redo in opposite direction */
      motor_setdir (MOTOR_IDX_2, MOTOR_DIR_BWD);
      motor_go (MOTOR_IDX_2, 50);
      motor_setdir (MOTOR_IDX_1, MOTOR_DIR_BWD);
      motor_go (MOTOR_IDX_1, 50);
      HAL_Delay (200);

      motor_go (MOTOR_IDX_1, 0);
      motor_go (MOTOR_IDX_2, 0);
      HAL_Delay (100);
#endif // test motors

      /* test encoders */
#if 0
      // do nothing but watch steps_motor1 and steps_motor2 evolution;
      distance1 = (((float)steps_motor1) / (float)ENCODER_PULSES_PER_SPIN) * WHEEL_PERIMETER;
      distance2 = (((float)steps_motor2) / (float)ENCODER_PULSES_PER_SPIN) * WHEEL_PERIMETER;
#endif // test encoders

      /* test serial */
#if 0
      char msg[] = "Hello world!\r\n";
      serial_write ((uint8_t*)msg, strlen(msg));
      int my_var  = 133;
      serial_write ((uint8_t*)&my_var, sizeof(my_var));
#endif // test serial

      /* check robot direction */
      if (chkup_pending == 1)
        {
          do_motor_correction ();
          chkup_pending = 0;
        }

      HAL_Delay (100);
    }

  return (-1);     // should not get here
}
