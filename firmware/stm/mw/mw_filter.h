///**
// * @file       mw_FILTERs.h
// * @copyright  [Your Copyright]
// * @license    [Your License]
// * @version    1.0.0
// * @date       2026-02-26
// * @author     Dong Son
// *
// * @brief
// */
///* Define to prevent recursive inclusion ------------------------------ */
//#ifndef INC_MW_FILTER_H_
//#define INC_MW_FILTER_H_
//
///* Includes ----------------------------------------------------------- */
//#include <stdint.h>
//#include <sys_config.h>
//#include <stdbool.h>
//#include "arm_math.h"
//#include <math.h>
//
///* Public defines ----------------------------------------------------- */
//#define MW_FILTER_WHEEL_RADIUS			WHEEL_RADIUS
//#define MW_FILTER_WHEEL_BASE			WHEEL_BASE
//#define MW_FILTER_PI					PI
//
///* Public enumerate/structure ----------------------------------------- */
//typedef struct
//{
//    float x;
//    float y;
//    float theta;
//    float b_omega;
//} ekf_state_t;
//
///* Kien truc Bo loc EKF dung CMSIS-DSP */
//typedef struct
//{
//    ekf_state_t state; // Vector X [4x1]
//} mw_filter_data_t;
//
///* Public macros ------------------------------------------------------ */
///* Public variables --------------------------------------------------- */
///* Public function prototypes ----------------------------------------- */
//void mw_filter_init(mw_filter_data_t *p_ekf,
//                    float x0, float y0, float theta0,
//                    float covx, float covy, float covtheta, float covb,
//                    float q0, float q1, float q2, float q3,
//                    float r_run, float r_stop);
//
//void mw_filter_predict(mw_filter_data_t *p_ekf, float d_s, float omega_raw, float dt);
//void mw_filter_update_run(mw_filter_data_t *p_ekf, float theta_enc);
//void mw_filter_update_stop(mw_filter_data_t *p_ekf, float omega_raw);
//
//#endif /* INC_MW_FILTER_H_ */
//
///* End of file -------------------------------------------------------- */
//
