/**
 * @file       drv8871.c
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-09
 * @author     Dong Son
 *
 * @brief
 */
/* Includes ----------------------------------------------------------- */
#include "drv8871.h"
#include "err.h"
#include <stddef.h>

/* Function definitions ----------------------------------------------- */
drv8871_err_t drv8871_init(drv8871_t *me)
{
    CHECK_PARAM((me == NULL), DRV8871_ERR_PARAM);

    CHECK_PARAM((me->tim_set_compare_channel1 == NULL), DRV8871_ERR_PARAM);
    CHECK_PARAM((me->tim_set_compare_channel2 == NULL), DRV8871_ERR_PARAM);

    me->tim_set_compare_channel1(0);
    me->tim_set_compare_channel2(0);

    return DRV8871_OK;
}

drv8871_err_t drv8871_set_pwm(drv8871_t *me, int16_t pwm)
{
	CHECK_PARAM((me == NULL), DRV8871_ERR_PARAM);

	if (pwm >= 0)
	{
		me->tim_set_compare_channel1((uint16_t)pwm);
		me->tim_set_compare_channel2(0);
	}
	else
	{
		me->tim_set_compare_channel1(0);
		me->tim_set_compare_channel2((uint16_t)(-pwm));
	}

	return DRV8871_OK;
}

/* End of file -------------------------------------------------------- */
