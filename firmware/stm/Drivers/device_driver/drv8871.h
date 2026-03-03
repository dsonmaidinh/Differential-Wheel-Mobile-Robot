/**
 * @file       drv8871.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-09
 * @author     Dong Son
 *
 * @brief
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef DEVICEDRIVERS_INC_DRV8871_H_
#define DEVICEDRIVERS_INC_DRV8871_H_

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
	DRV8871_OK = 0,
	DRV8871_ERR,
	DRV8871_ERR_PARAM,
} drv8871_err_t;

typedef struct
{
	void (*tim_set_compare_channel1)(uint16_t pwm);
	void (*tim_set_compare_channel2)(uint16_t pwm);
} drv8871_t;

/* Public function prototypes ----------------------------------------- */
drv8871_err_t drv8871_init(drv8871_t *me);

drv8871_err_t drv8871_set_pwm(drv8871_t *me, int16_t pwm);

#endif /* DEVICEDRIVERS_INC_DRV8871_H_ */

/* End of file -------------------------------------------------------- */
