/**
 * @file       mw_kinematics.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-26
 * @author     Dong Son
 *
 * @brief
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef INC_MW_KINEMATICS_H_
#define INC_MW_KINEMATICS_H_

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <sys_config.h>
#include <stdbool.h>

/* Public defines ----------------------------------------------------- */
#define MW_KINEMATIC_WHEEL_RADIUS		WHEEL_RADIUS
#define MW_KINEMATIC_WHEEL_BASE			WHEEL_BASE
#define MW_KINEMATICS_KV 				KV
#define MW_KINEMATICS_KW 				KW
#define MW_KINEMATICS_SAFETY_FACTOR 	SAFETY_FACTOR
#define MW_KINEMATICS_PI				PI
#define MW_KINEMATICS_USE_ODOMETRY		USE_ODOMETRY

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
	MW_KINEMATICS_OK = 0,    /**< Operation successful */
	MW_KINEMATICS_ERR_PARAM, /**< Invalid argument */
	MW_KINEMATICS_ERR        /**< General error */
} mw_kinematics_err_t;

typedef enum
{
	MW_KINEMATICS_CIRCLE	= STREAM_CIRCLE,
	MW_KINEMATICS_SQUARE	= STREAM_SQUARE,
	MW_KINEMATICS_TRIANGLE	= STREAM_TRIANGLE,
	MW_KINEMATICS_FIGURE8	= STREAM_FIGURE8,
	MW_KINEMATICS_PATH		= STREAM_PATH,
} mw_kinematics_shape_t;

typedef stream_tx_data_t 	mw_kinematics_data_t;

typedef struct {
    float x;
    float y;
    float theta;
} agv_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void mw_kinematics_init(bool *is_run);

mw_kinematics_err_t mw_kinematics_configurate_trajectory(mw_kinematics_data_t *me, uint8_t *trajectory_config);

void mw_kinematics_generate_trajectory(mw_kinematics_data_t *me, float t);

void mw_kinematics_inverse(float *target_omega_l, float *target_omega_r);

void mw_kinematics_forward(mw_kinematics_data_t *me, float dt, float *cur_omega_l, float *cur_omega_r, float d_l, float d_r);

#endif /* INC_MW_KINEMATICS_H_ */

/* End of file -------------------------------------------------------- */
