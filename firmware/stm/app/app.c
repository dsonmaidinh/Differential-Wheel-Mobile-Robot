/**
 * @file       app.c
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-03-02
 * @author     Dong Son
 *
 * @brief
 */
/* Includes ----------------------------------------------------------- */
#include <stdbool.h>
#include "stm32f1xx.h"
#include "app.h"
#include "sys_config.h"
#include "err.h"
#include "bsp_delay.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_esp.h"
#include "mw_controller.h"
#include "mw_kinematics.h"
#include "sys_task.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
	APP_STOP 		= BSP_ESP_CMD_STOP,
	APP_RUN  		= BSP_ESP_CMD_RUN,
	APP_RETURN_HOME = BSP_ESP_CMD_RETURN_HOME,
	APP_CONFIG 		= BSP_ESP_CMD_CONFIG,
	APP_ERR         = BSP_ESP_CMD_ERR,
	APP_NONE 		= BSP_ESP_CMD_NONE,
} app_state_t;

typedef mw_kinematics_data_t app_kinematics_data_t;
typedef error_info_t app_error_info_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
//--- Status
uint8_t w_status = 0;
uint8_t b_status = 0;

//--- Controller
float kp_r 			= 84.6381;
float ki_r 			= 2926.6282;
float kb_r 			= 34.5781;
float kd_r   		= 0;
float alpha_vel_r 	= 0.0f;
float kp_l 			= 83.9323;
float ki_l 			= 2730.3930;
float kb_l 			= 32.5309;
float kd_l   		= 0;
float alpha_vel_l 	= 0.0f;
pid_t pid_left 	= {0};
pid_t pid_right = {0};
int16_t pwm_left 	= 0;
int16_t pwm_right	= 0;

//--- Debug
app_error_info_t 		global_error = {0};

//--- FSM
app_state_t 			global_state = APP_STOP;

//--- Kinematics
app_kinematics_data_t	global_kinematics_data;
uint32_t time_pre = 0;
float target_omega_l = 0;
float target_omega_r = 0;
float global_t = 0.0f;
bool is_run = false;

//--- Motor
float left_vel	= 0;
float right_vel	= 0;
bool is_test = 0;

/* Private variables -------------------------------------------------- */
static int app_motion_task_id		= -1;
static int app_config_task_id		= -1;
static int app_test_motor_task_id	= -1;

/* Private function prototypes ---------------------------------------- */
static inline void app_task_motion_coltrol(void *arg);
static inline void app_task_process_data(void *arg);
static inline void app_task_test_motor(void *arg);
static inline void set_up_test_motor_task();

/* Function definitions ----------------------------------------------- */
uint8_t app_init(void)
{

	bsp_delay_init();
	bsp_esp_init(&w_status, &b_status);
	bsp_led_set_init(&w_status, &b_status);
	bsp_motor_init();

	mw_pid_set_params(&pid_left, kp_l, ki_l, kd_l, kb_l, alpha_vel_l);
	mw_pid_set_params(&pid_right, kp_r, ki_r, kd_r, kb_r, alpha_vel_r);

	mw_kinematics_init(&is_run);

	sys_task_init();
	app_motion_task_id 		= sys_task_add(app_task_motion_coltrol, NULL, 1000u, 0u);
	app_config_task_id		= sys_task_add(app_task_process_data, NULL, 100u, 0u);
	app_test_motor_task_id	= sys_task_add(app_task_test_motor, NULL, 10u, 0u);
	sys_task_start(app_motion_task_id);
	sys_task_start(app_config_task_id);

	return 0;
}

uint8_t app_process(void)
{
	sys_task_process();
	return 0;
}

/* Private definitions ------------------------------------------------ */
static inline void app_task_process_data(void *arg)
{
	app_state_t new_cmd = (app_state_t)bsp_esp_get_data();
	if(new_cmd == APP_NONE) return;
	global_state = new_cmd;
	switch(global_state)
	{
		case APP_STOP:
		case APP_NONE:
			is_run = false;
			sys_task_set_period(app_motion_task_id, 1000u);
			sys_task_run_now(app_motion_task_id);
			break;
		case APP_RETURN_HOME:
			set_up_test_motor_task();
			break;
		case APP_RUN:
			if (!is_run)
			{
				time_pre = HAL_GetTick();
				global_t = 0.0f;
			}
			is_run = true;
			sys_task_set_period(app_motion_task_id, 10u);
			sys_task_run_now(app_motion_task_id);
			break;
		case APP_CONFIG:
			is_run = false;
			mw_kinematics_configurate_trajectory(&global_kinematics_data, esp_rx_bufer);
			break;
		case APP_ERR:
			bsp_motor_stop();
			is_run = false;
			sys_task_set_period(app_motion_task_id, 1000u);
			CHECK_ERR(true, APP_ERR);
			break;
	}
}

static inline void app_task_motion_coltrol(void *arg)
{
	if(is_run == false)
	{
		bsp_motor_stop();
		return;
	}
	uint32_t time_now = HAL_GetTick();

	float dt = (float)(time_now - time_pre) / 1000.0f;
	if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;
	time_pre = time_now;

	global_t += dt;

	mw_kinematics_generate_trajectory(&global_kinematics_data, global_t);


	mw_kinematics_inverse(&target_omega_l, &target_omega_r);

	float current_omega_l = bsp_motor_get_vel(BSP_MOTOR_LEFT, dt);
	float current_omega_r = bsp_motor_get_vel(BSP_MOTOR_RIGHT, dt);

	pwm_left 	= mw_pid_vel(&pid_left, target_omega_l, current_omega_l, dt);
	pwm_right 	= mw_pid_vel(&pid_right, target_omega_r, current_omega_r, dt);

	bsp_motor_set_vel(BSP_MOTOR_LEFT, pwm_left);
	bsp_motor_set_vel(BSP_MOTOR_RIGHT, pwm_right);

	mw_kinematics_forward(&global_kinematics_data, dt, &current_omega_l, &current_omega_r);

	bsp_esp_send_data((uint8_t *) &global_kinematics_data);
}

static inline void app_task_test_motor(void *arg)
{

	uint32_t time_now = HAL_GetTick();

	float dt = (float)(time_now - time_pre) / 1000.0f;
	if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;
	time_pre = time_now;

	global_t += dt;

	left_vel  = bsp_motor_get_vel(BSP_MOTOR_LEFT, dt);
	right_vel = bsp_motor_get_vel(BSP_MOTOR_RIGHT, dt);

	pwm_left 	= mw_pid_vel(&pid_left, target_omega_l, left_vel, dt);
	pwm_right 	= mw_pid_vel(&pid_right, target_omega_r, right_vel, dt);

	bsp_motor_set_vel(BSP_MOTOR_LEFT, pwm_left);
	bsp_motor_set_vel(BSP_MOTOR_RIGHT, pwm_right);

}

static inline void set_up_test_motor_task()
{
	is_test = !is_test;
	// Reset
	target_omega_r = 0;
	target_omega_l = 0;
	time_pre = HAL_GetTick();
	global_t = 0.0f;

	mw_pid_set_params(&pid_left, kp_l, ki_l, kd_l, kb_l, alpha_vel_l);
	mw_pid_set_params(&pid_right, kp_r, ki_r, kd_r, kb_r, alpha_vel_r);

	bsp_motor_stop();

	if(is_test)
	{

		is_run = false;

		sys_task_stop(app_motion_task_id);
		sys_task_start(app_test_motor_task_id);

	}
	else
	{
		sys_task_stop(app_test_motor_task_id);
		sys_task_start(app_motion_task_id);
	}

}
/* End of file -------------------------------------------------------- */
