///**
// * @file       [Your File].c
// * @copyright  [Your Copyright]
// * @license    [Your License]
// * @version    1.0.0
// * @date       [Date]
// * @author     [Your Name]
// *
// * @brief
// */
///* Includes ----------------------------------------------------------- */
//#include "mw_filter.h"
//
///* Private defines ---------------------------------------------------- */
///* Private enumerate/structure ---------------------------------------- */
///* Kien truc Bo loc EKF dung CMSIS-DSP */
//typedef struct
//{
//    ekf_state_t state; // Vector X [4x1]
//
//    // Du lieu mang 1D (Phang hoa ma tran 4x4 thanh 16 phan tu)
//    float32_t P_data[16];
//    float32_t Q_data[16];
//
//    // Tham so nhieu do luong
//    float32_t R_run;
//    float32_t R_stop;
//
//    // Instances quan ly ma tran cua CMSIS-DSP
//    arm_matrix_instance_f32 mat_P;
//    arm_matrix_instance_f32 mat_Q;
//
//} filter_data_t;
//
///* Private macros ----------------------------------------------------- */
///* Public variables --------------------------------------------------- */
///* Private variables -------------------------------------------------- */
//filter_data_t ekf;
//
///* Private function prototypes ---------------------------------------- */
//static float normalize_angle(float angle);
//static void execute_joseph_update(filter_data_t *p_ekf, float32_t* K_data, float32_t* H_data, float R_val);
//
///* Function definitions ----------------------------------------------- */
//void mw_filter_init(mw_filter_data_t *p_ekf,
//                    float x0, float y0, float theta0,
//                    float covx, float covy, float covtheta, float covb,
//                    float q0, float q1, float q2, float q3,
//                    float r_run, float r_stop)
//{
//    // Khoi tao vector trang thai
//    ekf.state.x = x0;
//    ekf.state.y = y0;
//    ekf.state.theta = theta0;
//    ekf.state.b_omega = 0.0f;
//
//    // Khoi tao mang P va Q bang 0
//    for(int i = 0; i < 16; i++) {
//        ekf.P_data[i] = 0.0f;
//        ekf.Q_data[i] = 0.0f;
//    }
//
//    // Gan duong cheo chinh cho P
//    ekf.P_data[0]  = covx;      // P[0][0]
//    ekf.P_data[5]  = covy;      // P[1][1]
//    ekf.P_data[10] = covtheta;  // P[2][2]
//    ekf.P_data[15] = covb;      // P[3][3]
//
//    // Gan duong cheo chinh cho Q
//    ekf.Q_data[0]  = q0;        // Q[0][0]
//    ekf.Q_data[5]  = q1;        // Q[1][1]
//    ekf.Q_data[10] = q2;        // Q[2][2]
//    ekf.Q_data[15] = q3;        // Q[3][3]
//
//    // Luu R
//    ekf.R_run = r_run;
//    ekf.R_stop = r_stop;
//
//    // Khoi tao cac Instance ma tran CMSIS-DSP
//    arm_mat_init_f32(&ekf.mat_P, 4, 4, ekf.P_data);
//    arm_mat_init_f32(&ekf.mat_Q, 4, 4, ekf.Q_data);
//
//    if (p_ekf != NULL)
//    {
//		p_ekf->state = ekf.state;
//	}
//}
//
///* ==========================================================
// * 1. KHÂU PREDICT (Sử dụng Odometry ds và Góc IMU)
// * ========================================================== */
//void mw_filter_predict(mw_filter_data_t *p_ekf, float d_s, float omega_raw, float dt)
//{
//    // --- BƯỚC 1: TÍNH CÁC BIẾN TẠM ĐỂ TỐI ƯU TOÁN HỌC ---
//    // t1 = theta + (omega_raw - b_omega) * dt / 2
//    float t1 = ekf.state.theta + (omega_raw - ekf.state.b_omega) * dt * 0.5f;
//    float t2 = sinf(t1);
//    float t3 = cosf(t1);
//
//    // --- BƯỚC 2: CẬP NHẬT TRẠNG THÁI x_pred ---
//    ekf.state.x += d_s * t3;
//    ekf.state.y += d_s * t2;
//    ekf.state.theta = normalize_angle(ekf.state.theta + (omega_raw - ekf.state.b_omega) * dt);
//    // b_omega giu nguyen khong doi o buoc Predict
//
//    // --- BƯỚC 3: XÂY DỰNG MA TRẬN JACOBIAN F ---
//    float32_t F_data[16] = {0};
//    arm_matrix_instance_f32 mat_F;
//    arm_mat_init_f32(&mat_F, 4, 4, F_data);
//
//    F_data[0] = 1.0f;  F_data[1] = 0.0f;  F_data[2]  = -d_s * t2; F_data[3]  =  d_s * dt * t2 * 0.5f; // Hang 0 (Dung t2 - sin)
//	F_data[4] = 0.0f;  F_data[5] = 1.0f;  F_data[6]  =  d_s * t3; F_data[7]  = -d_s * dt * t3 * 0.5f; // Hang 1 (Dung t3 - cos)
//	F_data[8] = 0.0f;  F_data[9] = 0.0f;  F_data[10] = 1.0f;      F_data[11] = -dt;                   // Hang 2
//	F_data[12]= 0.0f;  F_data[13]= 0.0f;  F_data[14] = 0.0f;      F_data[15] = 1.0f;                  // Hang 3                 // Hang 3
//
//    // --- BƯỚC 4: TÍNH P = F*P*F^T + Q DÙNG CMSIS-DSP ---
//    float32_t F_T_data[16], temp1_data[16], temp2_data[16];
//    arm_matrix_instance_f32 mat_F_T, mat_temp1, mat_temp2;
//
//    arm_mat_init_f32(&mat_F_T, 4, 4, F_T_data);
//    arm_mat_init_f32(&mat_temp1, 4, 4, temp1_data);
//    arm_mat_init_f32(&mat_temp2, 4, 4, temp2_data);
//
//    arm_mat_trans_f32(&mat_F, &mat_F_T);                      // F^T
//    arm_mat_mult_f32(&mat_F, &ekf.mat_P, &mat_temp1);        // temp1 = F*P
//    arm_mat_mult_f32(&mat_temp1, &mat_F_T, &mat_temp2);       // temp2 = F*P*F^T
//    arm_mat_add_f32(&mat_temp2, &ekf.mat_Q, &ekf.mat_P);    // P = temp2 + Q
//
//    if (p_ekf != NULL)
//	{
//		p_ekf->state = ekf.state;
//	}
//}
//
//void mw_filter_update_run(mw_filter_data_t *p_ekf, float theta_enc)
//{
//    float32_t H_run[4] = {0.0f, 0.0f, 1.0f, 0.0f}; // H = [0, 0, 1, 0]
//
//    // y = z - h(x)
//    float y = normalize_angle(theta_enc - ekf.state.theta);
//
//    // Vi H = [0,0,1,0], H*P*H^T chinh la phan tu duong cheo P[2][2] (Index la 10)
//    float S = ekf.P_data[10] + ekf.R_run;
//    float S_inv = 1.0f / S;
//
//    // Tinh Kalman Gain K = P * H^T * S_inv (boc cot thu 3 cua P)
//    float32_t K_data[4] = {
//        ekf.P_data[2]  * S_inv,
//        ekf.P_data[6]  * S_inv,
//        ekf.P_data[10] * S_inv,
//        ekf.P_data[14] * S_inv
//    };
//
//    // Cap nhat state: X = X + K*y
//    ekf.state.x += K_data[0] * y;
//    ekf.state.y += K_data[1] * y;
//    ekf.state.theta = normalize_angle(ekf.state.theta + K_data[2] * y);
//    ekf.state.b_omega += K_data[3] * y;
//
//    // Cap nhat Ma tran Hiep phuong sai P (Joseph Form)
//    execute_joseph_update(&ekf, K_data, H_run, ekf.R_run);
//
//    if (p_ekf != NULL)
//	{
//		p_ekf->state = ekf.state;
//	}
//}
//
//
///* ==========================================================
// * 3. KHÂU UPDATE STOP - ZARU (Khi xe dung yen - Hoc Bias bang IMU)
// * ========================================================== */
//void mw_filter_update_stop(mw_filter_data_t *p_ekf, float omega_raw)
//{
//    float32_t H_stop[4] = {0.0f, 0.0f, 0.0f, 1.0f}; // H = [0, 0, 0, 1]
//
//    // y = z - h(x). Khi dung yen h(x) = b_omega
//    float y = omega_raw - ekf.state.b_omega;
//
//    // Vi H = [0,0,0,1], H*P*H^T chinh la phan tu duong cheo P[3][3] (Index la 15)
//    float S = ekf.P_data[15] + ekf.R_stop;
//    float S_inv = 1.0f / S;
//
//    // Tinh Kalman Gain K = P * H^T * S_inv (boc cot thu 4 cua P)
//    float32_t K_data[4] = {
//        ekf.P_data[3]  * S_inv,
//        ekf.P_data[7]  * S_inv,
//        ekf.P_data[11] * S_inv,
//        ekf.P_data[15] * S_inv
//    };
//
//    // O che do Stop, chung ta chi cap nhat b_omega
//    // X, Y, Theta cua xe khong duoc phep thay doi!
//    ekf.state.b_omega += K_data[3] * y;
//
//    // Cap nhat Ma tran Hiep phuong sai P (Joseph Form)
//    execute_joseph_update(&ekf, K_data, H_stop, ekf.R_stop);
//
//    if (p_ekf != NULL)
//	{
//		p_ekf->state = ekf.state;
//	}
//}
//
///* Private definitions ------------------------------------------------ */
//static float normalize_angle(float angle)
//{
//	angle = fmodf(angle, 2.0f * MW_FILTER_PI);
//	if (angle >  MW_FILTER_PI) angle -= 2.0f * MW_FILTER_PI;
//	if (angle < -MW_FILTER_PI) angle += 2.0f * MW_FILTER_PI;
//	return angle;
//}
//
//static void execute_joseph_update(filter_data_t *p_ekf, float32_t* K_data, float32_t* H_data, float R_val)
//{
//    // Khai bao cac ma tran can thiet
//    arm_matrix_instance_f32 mat_K, mat_H, mat_KH, mat_I, mat_I_KH, mat_I_KHt, mat_temp1, mat_temp2;
//    arm_matrix_instance_f32 mat_Kt, mat_KR, mat_KRKt;
//
//    float32_t KH_data[16], I_data[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
//    float32_t I_KH_data[16], I_KHt_data[16], temp1_data[16], temp2_data[16];
//    float32_t Kt_data[4], KR_data[4], KRKt_data[16];
//
//    // Khoi tao instance
//    arm_mat_init_f32(&mat_K, 4, 1, K_data);
//    arm_mat_init_f32(&mat_H, 1, 4, H_data);
//    arm_mat_init_f32(&mat_KH, 4, 4, KH_data);
//    arm_mat_init_f32(&mat_I, 4, 4, I_data);
//    arm_mat_init_f32(&mat_I_KH, 4, 4, I_KH_data);
//    arm_mat_init_f32(&mat_I_KHt, 4, 4, I_KHt_data);
//    arm_mat_init_f32(&mat_temp1, 4, 4, temp1_data);
//    arm_mat_init_f32(&mat_temp2, 4, 4, temp2_data);
//    arm_mat_init_f32(&mat_Kt, 1, 4, Kt_data);
//    arm_mat_init_f32(&mat_KR, 4, 1, KR_data);
//    arm_mat_init_f32(&mat_KRKt, 4, 4, KRKt_data);
//
//    // 1. Tinh (I - KH)
//    arm_mat_mult_f32(&mat_K, &mat_H, &mat_KH);
//    arm_mat_sub_f32(&mat_I, &mat_KH, &mat_I_KH);
//
//    // 2. Tinh (I - KH) * P * (I - KH)^T
//    arm_mat_trans_f32(&mat_I_KH, &mat_I_KHt);
//    arm_mat_mult_f32(&mat_I_KH, &p_ekf->mat_P, &mat_temp1);
//    arm_mat_mult_f32(&mat_temp1, &mat_I_KHt, &mat_temp2);
//
//    // 3. Tinh K * R * K^T
//    arm_mat_scale_f32(&mat_K, R_val, &mat_KR);
//    arm_mat_trans_f32(&mat_K, &mat_Kt);
//    arm_mat_mult_f32(&mat_KR, &mat_Kt, &mat_KRKt);
//
//    // 4. P_upd = [temp2] + [KRKt]
//    arm_mat_add_f32(&mat_temp2, &mat_KRKt, &p_ekf->mat_P);
//}
//
///* End of file -------------------------------------------------------- */
