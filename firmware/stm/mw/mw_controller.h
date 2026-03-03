/**
 * @file       mw_controller.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-26
 * @author     Dong Son
 *
 * @brief
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef INC_MW_CONTROLLER_H_
#define INC_MW_CONTROLLER_H_

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include "sys_config.h"

/* Public defines ----------------------------------------------------- */
#define MW_PID_MAX_OUTPUT 		PID_MAX_OUTPUT

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
	MW_CONTROLLER_OK = 0,
	MW_CONTROLLER_ERR,
	MW_CONTROLLER_ERR_PARAM,
} pid_motor_err_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float kb;
    float alpha;
	float e_sat;
	float UI_pre;
	float ek_pre;
    float UD_func_pre;
} pid_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
pid_motor_err_t mw_pid_set_params(pid_t *pid, float kp, float ki, float kd, float kb, float alpha);

int16_t mw_pid_vel(pid_t *pid, float desired_value, float current_value, float dt);

#endif /* INC_MW_CONTROLLER_H_ */

/* End of file -------------------------------------------------------- */
