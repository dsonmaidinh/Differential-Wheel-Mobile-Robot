/**
 * @file       bsp_delay.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-25
 * @author     Dong Son
 *
 * @brief      Delay functionality header file
 * @note       This file provides the interface for the delay functionality.
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef BSP_DELAY_H_
#define BSP_DELAY_H_

/* Includes ----------------------------------------------------------- */
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  BSP_DELAY_OK = 0,
  BSP_DELAY_ERR
} bsp_delay_err_t;

/* Public function prototypes ----------------------------------------- */
bsp_delay_err_t bsp_delay_init(void);
void        bsp_delay_ms(uint32_t ms);

#endif /* INC_BSP_DELAY_H_ */

/* End of file -------------------------------------------------------- */
