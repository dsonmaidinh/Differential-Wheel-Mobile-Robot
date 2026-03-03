/**
 * @file       mw_controller.c
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-26
 * @author     Dong Son
 *
 * @brief
 */
/* Includes ----------------------------------------------------------- */
#include "mw_controller.h"
#include "err.h"
#include <stddef.h>
#include <stdint.h>

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
pid_motor_err_t mw_pid_set_params(pid_t *pid, float kp, float ki, float kd, float kb, float alpha)
{
	CHECK_PARAM((pid == NULL), MW_CONTROLLER_ERR_PARAM);

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->kb = kb;
	pid->alpha = alpha;

	pid->e_sat = 0;
	pid->UI_pre = 0;
	pid->ek_pre = 0;
	pid->UD_func_pre = 0;

	return MW_CONTROLLER_OK;
}

int16_t mw_pid_vel(pid_t *pid, float desired_value, float current_value, float dt)
{
	CHECK_PARAM((pid == NULL), MW_CONTROLLER_ERR_PARAM);

    float pidterm = 0, pid_sat = 0;
    float ek;
    float UP, UI;

    int16_t pidout;
    if(pid->ki == 0) pid->kb = 0;

    ek = desired_value - current_value;

    UP = pid->kp * ek;
    UI = pid->UI_pre + pid->ki * ek * dt + pid->kb * pid->e_sat * dt;
    pidterm = UP + UI;

    /* Saturation */
    if(pidterm >= MW_PID_MAX_OUTPUT) {
        pid_sat = MW_PID_MAX_OUTPUT;
        pid->e_sat = MW_PID_MAX_OUTPUT - pidterm;
    }
    else if (pidterm <= -MW_PID_MAX_OUTPUT) {
        pid_sat = -MW_PID_MAX_OUTPUT;
        pid->e_sat = -MW_PID_MAX_OUTPUT - pidterm;
    }
    else {
        pid_sat = pidterm;
        pid->e_sat = 0;
    }

    pid->ek_pre = ek;
    pid->UI_pre = UI;

    pidout = (int16_t) pid_sat;
    return pidout;
}
/* Private definitions ------------------------------------------------ */
/* End of file -------------------------------------------------------- */
