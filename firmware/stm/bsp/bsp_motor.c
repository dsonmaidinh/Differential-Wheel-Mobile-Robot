/**
 * @file       bsp_motor.c
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-09
 * @author     Dong Son
 *
 * @brief
 */
/* Includes ----------------------------------------------------------- */
#include "bsp_motor.h"
#include "drv8871.h"
#include "err.h"

/* Private defines ---------------------------------------------------- */
#define BSP_MOTOR_CONVERSION_FACTOR	(2.0f * BSP_MOTOR_PI) / (BSP_MOTOR_ENCODER_PPR)
#define BSP_TIM_PWM_HANDLE 		htim3
#define BSP_TIM_ENCODERL_HANDLE htim1
#define BSP_TIM_ENCODERR_HANDLE htim8

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
	TIM_HandleTypeDef *pwm_tim;
	TIM_HandleTypeDef *encoder_tim;
	uint32_t pwm_channel[2];

	drv8871_t motor_driver;

	bsp_motor_data_t data;
} bsp_motor_t;
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern TIM_HandleTypeDef BSP_TIM_PWM_HANDLE;
extern TIM_HandleTypeDef BSP_TIM_ENCODERL_HANDLE;
extern TIM_HandleTypeDef BSP_TIM_ENCODERR_HANDLE;

/* Private variables -------------------------------------------------- */
static bsp_motor_t motors[BSP_MOTOR_COUNT];

/* Private function prototypes ---------------------------------------- */
static inline void bsp_motor_get_counter(bsp_motor_id_t id);
static inline void bsp_motor_cal_vel(bsp_motor_id_t id, float dt);
static void motor_left_set_compare_channel1(uint16_t pwm);
static void motor_left_set_compare_channel2(uint16_t pwm);
static void motor_right_set_compare_channel1(uint16_t pwm);
static void motor_right_set_compare_channel2(uint16_t pwm);

/* Function definitions ----------------------------------------------- */
bsp_motor_err_t bsp_motor_init(void)
{
	bsp_motor_t *p_left 	= &motors[BSP_MOTOR_LEFT];
	p_left->pwm_tim         = &BSP_TIM_PWM_HANDLE;
	p_left->encoder_tim     = &BSP_TIM_ENCODERL_HANDLE;
	p_left->pwm_channel[0]  = TIM_CHANNEL_3;
	p_left->pwm_channel[1]  = TIM_CHANNEL_4;
	p_left->motor_driver.tim_set_compare_channel1 = motor_left_set_compare_channel1;
	p_left->motor_driver.tim_set_compare_channel2 = motor_left_set_compare_channel2;
	drv8871_init(&p_left->motor_driver);
	HAL_TIM_PWM_Start(p_left->pwm_tim, p_left->pwm_channel[0]);
	HAL_TIM_PWM_Start(p_left->pwm_tim, p_left->pwm_channel[1]);
	__HAL_TIM_SET_COMPARE(p_left->pwm_tim, p_left->pwm_channel[0], 0);
	__HAL_TIM_SET_COMPARE(p_left->pwm_tim, p_left->pwm_channel[1], 0);
	HAL_TIM_Encoder_Start(p_left->encoder_tim, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(p_left->encoder_tim, 0);

	bsp_motor_t *p_right 	= &motors[BSP_MOTOR_RIGHT];
	p_right->pwm_tim        = &BSP_TIM_PWM_HANDLE;
	p_right->encoder_tim    = &BSP_TIM_ENCODERR_HANDLE;
	p_right->pwm_channel[0] = TIM_CHANNEL_2;
	p_right->pwm_channel[1] = TIM_CHANNEL_1;
	p_right->motor_driver.tim_set_compare_channel1 = motor_right_set_compare_channel1;
	p_right->motor_driver.tim_set_compare_channel2 = motor_right_set_compare_channel2;
	drv8871_init(&p_right->motor_driver);
	HAL_TIM_PWM_Start(p_right->pwm_tim, p_right->pwm_channel[0]);
	HAL_TIM_PWM_Start(p_right->pwm_tim, p_right->pwm_channel[1]);
	__HAL_TIM_SET_COMPARE(p_right->pwm_tim, p_right->pwm_channel[0], 0);
	__HAL_TIM_SET_COMPARE(p_right->pwm_tim, p_right->pwm_channel[1], 0);
	HAL_TIM_Encoder_Start(p_right->encoder_tim, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(p_right->encoder_tim, 0);

	return BSP_MOTOR_OK;
}

bsp_motor_err_t bsp_motor_find_params(void)
{
	bsp_motor_init();
	bsp_motor_t *p_left 	= &motors[BSP_MOTOR_LEFT];
	bsp_motor_t *p_right 	= &motors[BSP_MOTOR_RIGHT];
	__HAL_TIM_SET_COMPARE(p_left->pwm_tim, p_left->pwm_channel[0], __HAL_TIM_GET_AUTORELOAD(p_left->pwm_tim));
	__HAL_TIM_SET_COMPARE(p_right->pwm_tim, p_right->pwm_channel[0], __HAL_TIM_GET_AUTORELOAD(p_right->pwm_tim));

	return BSP_MOTOR_OK;
}

bsp_motor_err_t bsp_motor_get_data(bsp_motor_id_t id, bsp_motor_data_t *p_data_out, float dt)
{
     CHECK_PARAM((id >= BSP_MOTOR_COUNT || p_data_out != NULL), BSP_MOTOR_ERR_PARAM);

     bsp_motor_cal_vel(id, dt);

     *p_data_out = motors[id].data;

     return BSP_MOTOR_OK;
}

bsp_motor_err_t bsp_motor_get_data_kinematics(bsp_motor_id_t id, float dt, float *omega, float *d)
{
     CHECK_PARAM((id >= BSP_MOTOR_COUNT ), BSP_MOTOR_ERR_PARAM);

     bsp_motor_cal_vel(id, dt);

     *omega = motors[id].data.cur_omega;
     *d		= motors[id].data.d;

     return BSP_MOTOR_OK;
}

float bsp_motor_get_vel(bsp_motor_id_t id, float dt)
{
	bsp_motor_cal_vel(id, dt);
	return motors[id].data.cur_omega;
}

float bsp_motor_get_round(bsp_motor_id_t id, float dt)
{
	bsp_motor_cal_vel(id, dt);
	return motors[id].data.cntPos * 360.0f;
}

bsp_motor_err_t bsp_motor_set_vel(bsp_motor_id_t id, int16_t pwm)
{
	CHECK_PARAM((id >= BSP_MOTOR_COUNT), BSP_MOTOR_ERR_PARAM);

	drv8871_set_pwm(&motors[id].motor_driver, pwm);

	return BSP_MOTOR_OK;
}

bsp_motor_err_t bsp_motor_stop(void)
{
	bsp_motor_t *p_left 	= &motors[BSP_MOTOR_LEFT];
	bsp_motor_t *p_right 	= &motors[BSP_MOTOR_RIGHT];
	__HAL_TIM_SET_COMPARE(p_left->pwm_tim, p_left->pwm_channel[0], 0);
	__HAL_TIM_SET_COMPARE(p_left->pwm_tim, p_left->pwm_channel[1], 0);
	__HAL_TIM_SET_COMPARE(p_right->pwm_tim, p_right->pwm_channel[0], 0);
	__HAL_TIM_SET_COMPARE(p_right->pwm_tim, p_right->pwm_channel[1], 0);
	return BSP_MOTOR_OK;
}

/* Private definitions ------------------------------------------------ */
static inline void bsp_motor_get_counter(bsp_motor_id_t id)
{
	uint16_t currentCounter = (uint16_t)__HAL_TIM_GET_COUNTER(motors[id].encoder_tim);

	int16_t delta = (int16_t)(currentCounter - motors[id].data.lastCounterVal);

	motors[id].data.totalTicks += delta;

	motors[id].data.cntVel = delta;

	motors[id].data.d = (float)delta*(float)BSP_WHEEL_TRACK;

	motors[id].data.total_distance += motors[id].data.d;

	if (BSP_MOTOR_ENCODER_PPR > 0)
	{
		motors[id].data.cntPos = (float)motors[id].data.totalTicks / (float)BSP_MOTOR_ENCODER_PPR;
	}

	motors[id].data.lastCounterVal = currentCounter;
}

static inline void bsp_motor_cal_vel(bsp_motor_id_t id, float dt)
{
	bsp_motor_get_counter(id);

	if (dt > 0.0001f)
	{
		motors[id].data.cur_omega = (motors[id].data.cntVel * BSP_MOTOR_CONVERSION_FACTOR) / dt;
	}
	else
	{
		motors[id].data.cur_omega = 0.0f;
	}
}

static void motor_left_set_compare_channel1(uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(motors[BSP_MOTOR_LEFT].pwm_tim, motors[BSP_MOTOR_LEFT].pwm_channel[0], pwm);
}

static void motor_left_set_compare_channel2(uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(motors[BSP_MOTOR_LEFT].pwm_tim, motors[BSP_MOTOR_LEFT].pwm_channel[1], pwm);
}

static void motor_right_set_compare_channel1(uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(motors[BSP_MOTOR_RIGHT].pwm_tim, motors[BSP_MOTOR_RIGHT].pwm_channel[0], pwm);
}

static void motor_right_set_compare_channel2(uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(motors[BSP_MOTOR_RIGHT].pwm_tim, motors[BSP_MOTOR_RIGHT].pwm_channel[1], pwm);
}

/* End of file -------------------------------------------------------- */
