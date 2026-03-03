/**
 * @file       bsp_motor.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-09
 * @author     Dong Son
 *
 * @brief
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef DEVICEDRIVERS_INC_BSP_MOTOR_H_
#define DEVICEDRIVERS_INC_BSP_MOTOR_H_

/* Includes ----------------------------------------------------------- */
#include "drv8871.h"
#include "stm32f1xx_hal.h"
#include "sys_config.h"

/* Public defines ----------------------------------------------------- */
#define BSP_MOTOR_PI                    PI
#define BSP_MOTOR_ENCODER_PPR			MOTOR_ENCODER_PPR
#define BSP_MOTOR_COUNT 				MOTOR_COUNT
#define BSP_WHEEL_TRACK					WHEEL_TRACK_FACTOR

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
	BSP_MOTOR_OK			= DRV8871_OK,
	BSP_MOTOR_ERR			= DRV8871_ERR,
	BSP_MOTOR_ERR_PARAM		= DRV8871_ERR_PARAM,
} bsp_motor_err_t;

typedef enum
{
	BSP_MOTOR_RIGHT = 0,
    BSP_MOTOR_LEFT = 1,
} bsp_motor_id_t;

typedef struct
{
	float cur_omega;
	float vel_conversion_factor;

	float  cntPos;          // Số vòng quay (Revolution Count)
	int16_t  cntValue;        // Giá trị xung hiện tại (0 đến ARR)
	int16_t  cntVel;          // Vận tốc (tính bằng delta ticks)
	uint16_t encoderPPR;

	int32_t  lastTotalTicks;
	int64_t  totalTicks;
	uint16_t lastCounterVal;
	uint32_t lastTime;
	uint32_t timer_arr;       // Lưu giá trị ARR để xử lý tràn

	float d;
	float total_distance;
} bsp_motor_data_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

bsp_motor_err_t bsp_motor_init(void);

bsp_motor_err_t bsp_motor_find_params(void);

bsp_motor_err_t bsp_motor_get_data(bsp_motor_id_t id, bsp_motor_data_t *p_data_out, float dt);

bsp_motor_err_t bsp_motor_get_data_kinematics(bsp_motor_id_t id, float dt, float *omega, float *d);

float bsp_motor_get_vel(bsp_motor_id_t id, float dt);

float bsp_motor_get_round(bsp_motor_id_t id, float dt);

bsp_motor_err_t bsp_motor_set_vel(bsp_motor_id_t id, int16_t pwm);

bsp_motor_err_t bsp_motor_stop(void);

#endif /* DEVICEDRIVERS_INC_BSP_MOTOR_H_ */

/* End of file -------------------------------------------------------- */
