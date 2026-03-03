/**
 * @file       bsp_delay.c
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-25
 * @author     Dong Son
 *
 * @brief       This file implements the delay functionality using TIM9 for microsecond delays.
 * @note        This implementation is based on the STM32F4xx HAL library.
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_delay.h"
#include "err.h"

/* Private defines ---------------------------------------------------- */
#define BSP_DELAY_TIM_HANDLE htim4

/* Public variables --------------------------------------------------- */
extern TIM_HandleTypeDef BSP_DELAY_TIM_HANDLE;

/* Private function prototypes ---------------------------------------- */
static void delay_wait_us_(uint32_t us);

/* Function definitions ----------------------------------------------- */
bsp_delay_err_t bsp_delay_init(void)
{
	CHECK_ERR(HAL_TIM_Base_Start(&BSP_DELAY_TIM_HANDLE) != HAL_OK, BSP_DELAY_ERR);

	return BSP_DELAY_OK;
}

void bsp_delay_ms(uint32_t ms)
{
  while (ms--)
  {
    delay_wait_us_(1000);
  }
}

/* Private functions --------------------------------------------------------- */
static void delay_wait_us_(uint32_t us)
{
  uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&BSP_DELAY_TIM_HANDLE);

  while ((uint16_t)(__HAL_TIM_GET_COUNTER(&BSP_DELAY_TIM_HANDLE) - start) < us)
  {
    __NOP();
  }
}

/* End of file -------------------------------------------------------- */
