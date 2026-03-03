/**
 * @file       mw_kinematics.c
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-26
 * @author     Dong Son
 *
 * @brief
 */
/* Includes ----------------------------------------------------------- */
#include "mw_kinematics.h"
#include "err.h"
#include <math.h>
#include <stddef.h>

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
typedef struct {
    float x;
    float y;
} point_t;

typedef struct
{
	float omega;
	float velocity;
	float T;
	float param;
	float half_side;
	float t_per_side;
} trajectory_params_t;

typedef struct
{
	point_t 	points[MAX_PATH_POINTS];
	uint16_t 	total_points;
	uint16_t 	current_index;
	int8_t 		direction;
	uint8_t 	is_cycle;
	bool		*is_run;
	float 		velocity;
	float 		start_x;
	float 		start_y;
} path_params_t;

typedef struct
{
    uint8_t 	header[2];
    uint8_t 	cmd;
} command_t;

typedef struct
{
	float theta;
	float omega;
	float velocity;
} kinematics_data_t;

typedef struct
{
	kinematics_data_t target;
	kinematics_data_t actual;
	float e_pos;
	float e_theta;
} kinematics_t;

typedef stream_rx_trajectory_config_t 	trajectory_config_t;
typedef stream_rx_path_config_t 		path_config_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
kinematics_t kinematics;

/* Private variables -------------------------------------------------- */
uint8_t shape = MW_KINEMATICS_CIRCLE;
trajectory_params_t 				trajectory_handle;
path_params_t						path_handle;

/* Private function prototypes ---------------------------------------- */
static inline void set_trajectory(uint8_t *params);
static inline void set_path(mw_kinematics_data_t *me, uint8_t *params);
static inline void calculate_trajectory();
static inline void calculate_error(mw_kinematics_data_t *me);
static inline void calculate_set_point(mw_kinematics_data_t *me);
static inline void generate_circular_trajectory(mw_kinematics_data_t *me, float t);
static inline void generate_square_trajectory(mw_kinematics_data_t *me, float t);
static inline void generate_triangle_trajectory(mw_kinematics_data_t *me, float t);
static inline void generate_figure8_trajectory(mw_kinematics_data_t *me, float t);
static inline void generate_path_trajectory(mw_kinematics_data_t *me, float t);
static inline void normalize_angle(float *p_angle);

typedef void (*p_trajectory_handler)(mw_kinematics_data_t *me, float t);
p_trajectory_handler my_trajectory[] =
{
	generate_circular_trajectory,
	generate_square_trajectory,
	generate_triangle_trajectory,
	generate_figure8_trajectory,
	generate_path_trajectory,
};

/* Function definitions ----------------------------------------------- */
void mw_kinematics_init(bool *is_run)
{
	path_handle.is_run = is_run;
}

mw_kinematics_err_t mw_kinematics_configurate_trajectory(mw_kinematics_data_t *me, uint8_t *trajectory_config)
{
	me->Pdx = 0;
	me->Pdy = 0;

	command_t *cmd_handle = (command_t *) trajectory_config;
	switch(cmd_handle->cmd)
	{
		case STREAM_CMD_SET_TRAJECTORY:
			set_trajectory(trajectory_config);
			calculate_trajectory();
			break;
		case STREAM_CMD_SET_PATH:
			set_path(me, trajectory_config);
			break;
		case STREAM_CMD_NONE:
		default:
			break;
	}

	return MW_KINEMATICS_OK;
}

void mw_kinematics_generate_trajectory(mw_kinematics_data_t *me, float t)
{
	my_trajectory[shape](me, t);

	if(shape != MW_KINEMATICS_PATH)
	{
		calculate_error(me);
	}

	calculate_set_point(me);
}

void mw_kinematics_inverse(float *target_omega_l, float *target_omega_r)
{
	// ω_R = (2v + ωD) / (2R)
	// ω_L = (2v - ωD) / (2R)
    float v_des = kinematics.target.velocity;
    float w_des = kinematics.target.omega;

    float v_linear_component 	= v_des / MW_KINEMATIC_WHEEL_RADIUS;
    float w_angular_component 	= (w_des * MW_KINEMATIC_WHEEL_BASE) / (2.0f * MW_KINEMATIC_WHEEL_RADIUS);

    *(target_omega_r) = v_linear_component + w_angular_component;
    *(target_omega_l) = v_linear_component - w_angular_component;
}

void mw_kinematics_forward(mw_kinematics_data_t *me, float dt, float *cur_omega_l, float *cur_omega_r)
{
	// v = R * (wR + wL) / 2
	// w = R * (wR - wL) / Base
    float motor_omega_r = *(cur_omega_r);
    float motor_omega_l = *(cur_omega_l);
    float theta 		= kinematics.actual.theta;
    float v 			= (MW_KINEMATIC_WHEEL_RADIUS * (motor_omega_r + motor_omega_l)) * 0.5f;
    float w 			= (MW_KINEMATIC_WHEEL_RADIUS * (motor_omega_r - motor_omega_l)) / MW_KINEMATIC_WHEEL_BASE;

    // debug
    kinematics.actual.velocity 	= v;
    kinematics.actual.omega 	= w;

    if ((fabsf(w) < 1e-6f))
    {
        me->Px 		+= v * cosf(theta) * dt;
        me->Py		+= v * sinf(theta) * dt;
        kinematics.actual.theta 	+= w * dt;
    }
    else
    {
        float R = v / w;
        float d_theta = w * dt;
        float theta_new = theta + d_theta;
        me->Px += R * (sinf(theta_new) - sinf(theta));
        me->Py -= R * (cosf(theta_new) - cosf(theta));
        kinematics.actual.theta 	+= w * dt;
    }

    normalize_angle(&kinematics.actual.theta);
}

/* Private definitions ------------------------------------------------ */
static inline void set_trajectory(uint8_t *params)
{
	trajectory_config_t *trajectory_param = (trajectory_config_t *)params;
	shape 						= trajectory_param->shape;
	trajectory_handle.param 	= trajectory_param->param;
	trajectory_handle.velocity 	= trajectory_param->velocity;
}

static inline void set_path(mw_kinematics_data_t *me, uint8_t *params)
{
	shape = MW_KINEMATICS_PATH;

	path_config_t *path_param = (path_config_t*)params;

	uint16_t count = path_param->count;

	if (count > MAX_PATH_POINTS)
	{
		count = MAX_PATH_POINTS;
	}

	path_handle.is_cycle 			= path_param->is_cycle;
	path_handle.velocity 			= path_param->velocity;
	path_handle.start_x 			= path_param->start_x;
	path_handle.start_y 			= path_param->start_y;
	path_handle.total_points 		= count;
	path_handle.current_index 		= 0;
	path_handle.direction 			= 1;

	float *p_float_data = (float*)(params + sizeof(path_config_t));

	for (uint16_t i = 0; i < count; i++)
	{
		path_handle.points[i].x = p_float_data[i * 2];
		path_handle.points[i].y = p_float_data[i * 2 + 1];
	}

	me->Pdx = path_handle.start_x;
	me->Pdy = path_handle.start_y;
	me->Px = path_handle.start_x;
	me->Py = path_handle.start_y;
}


static inline void calculate_trajectory()
{
	float param 	= trajectory_handle.param;
	float velocity 	= trajectory_handle.velocity;
	switch(shape)
	{
		case MW_KINEMATICS_CIRCLE:
		{
			float R = fabsf(param);
			float D = MW_KINEMATIC_WHEEL_BASE;
			if (R > 0.001f)
			{
				 float scale_factor = R / (R + D / 2.0f);
				 velocity *= scale_factor;
				 trajectory_handle.omega 	= velocity / R;
			}
			else
			{
				 trajectory_handle.omega = 0;
			}
			break;
		}

		case MW_KINEMATICS_SQUARE:
			trajectory_handle.half_side 	= param / 2.0f;
			trajectory_handle.t_per_side 	= param / velocity;
			trajectory_handle.T 			= 4.0f * trajectory_handle.t_per_side;
			break;

		case MW_KINEMATICS_TRIANGLE:
			trajectory_handle.half_side 	= param / 2.0f;
			trajectory_handle.t_per_side 	= param / velocity;
			trajectory_handle.T 			= 3.0f * trajectory_handle.t_per_side;
			break;

		case MW_KINEMATICS_FIGURE8:
			float R = fabsf(param);
			if (R > 0.001f)
			{
				trajectory_handle.omega = velocity / R;
			}
			else
			{
				trajectory_handle.omega = 0;
			}
			break;
	}
}

static inline void generate_circular_trajectory(mw_kinematics_data_t *me, float t)
{
	float R 	= trajectory_handle.param;
	float omega = trajectory_handle.omega;
	float angle = omega * t;

	me->Pdx = R * cosf(angle);
	me->Pdy = R * sinf(angle);

//	kinematics.target.theta = angle + (MW_KINEMATICS_PI / 2.0f);
}

static inline void generate_square_trajectory(mw_kinematics_data_t *me, float t)
{
	float a_half 		= trajectory_handle.half_side;
	float t_cycle 		= fmodf(t, trajectory_handle.T);
	int current_side 	= (int)(t_cycle / trajectory_handle.t_per_side);
	float t_local 		= t_cycle - (current_side * trajectory_handle.t_per_side);
	float dist_local 	= trajectory_handle.velocity * t_local;

	switch(current_side)
	{
		case 0: // Cạnh 1: Đi lên (x cố định dương, y tăng)
			// Từ (a/2, -a/2) -> (a/2, a/2)
			me->Pdx = a_half;
			me->Pdy = -a_half + dist_local;
//			kinematics.target.theta = MW_KINEMATICS_PI / 2.0f;
			break;

		case 1: // Cạnh 2: Đi sang trái (y cố định dương, x giảm)
			// Từ (a/2, a/2) -> (-a/2, a/2)
			me->Pdx = a_half - dist_local;
			me->Pdy = a_half;
//			kinematics.target.theta = MW_KINEMATICS_PI;
			break;

		case 2: // Cạnh 3: Đi xuống (x cố định âm, y giảm)
			// Từ (-a/2, a/2) -> (-a/2, -a/2)
			me->Pdx = -a_half;
			me->Pdy = a_half - dist_local;
//			kinematics.target.theta = -MW_KINEMATICS_PI / 2.0f;
			break;

		case 3: // Cạnh 4: Đi sang phải (y cố định âm, x tăng)
			// Từ (-a/2, -a/2) -> (a/2, -a/2)
			me->Pdx = -a_half + dist_local;
			me->Pdy = -a_half;
//			kinematics.target.theta = 0.0f;
			break;

		default: // Dự phòng (không bao giờ xảy ra nếu tính đúng)
			me->Pdx = a_half;
			me->Pdy = -a_half;
			break;
	}
}

static inline void generate_triangle_trajectory(mw_kinematics_data_t *me, float t)
{
	float a_half 		= trajectory_handle.half_side;
	float t_per_side 	= trajectory_handle.t_per_side;
	float v 			= trajectory_handle.velocity;
	float t_cycle 		= fmodf(t, trajectory_handle.T);
	int current_side 	= (int)(t_cycle / t_per_side);
	float t_local 		= t_cycle - (current_side * t_per_side);
	float dist 			= v * t_local;

	float sqrt3 = 1.73205f;
	float r_circum = a_half * 2.0f * sqrt3 / 3.0f; // a * sqrt3 / 3
	float r_inner  = a_half * 2.0f * sqrt3 / 6.0f; // a * sqrt3 / 6

	switch(current_side)
	{
		case 0: // Cạnh 1: Lên chéo phải (Góc 60 độ)
			// Từ (-a/2, -r_inner) đi lên
			me->Pdx = -a_half + dist * 0.5f;
			me->Pdy = -r_inner + dist * sqrt3 / 2.0f;
//			kinematics.target.theta = MW_KINEMATICS_PI / 3.0f;
			break;

		case 1: // Cạnh 2: Xuống chéo phải (Góc -60 độ)
			// Từ (0, r_circum) đi xuống
			me->Pdx = 0.0f + dist * 0.5f;
			me->Pdy = r_circum - dist * sqrt3 / 2.0f;
//			kinematics.target.theta = -MW_KINEMATICS_PI / 3.0f;
			break;

		case 2: // Cạnh 3: Sang trái (Góc 180 độ)
			// Từ (a/2, -r_inner) sang trái
			me->Pdx = a_half - dist;
			me->Pdy = -r_inner;
//			kinematics.target.theta = MW_KINEMATICS_PI;
			break;

		default:
			 // Dự phòng: Về tâm hoặc đứng yên
			me->Pdx = 0;
			me->Pdy = 0;
			break;
	}
}

static inline void generate_figure8_trajectory(mw_kinematics_data_t *me, float t)
{
	float A 		= trajectory_handle.param;
	float omega 	= trajectory_handle.omega;
	float phase 	= omega * t;
	float sin_val 	= sinf(phase);
	float cos_val 	= cosf(phase); 			// cos(wt)
	float cos_2val 	= cosf(2.0f * phase); 	// cos(2wt) - Dùng cho đạo hàm y

	me->Pdx = A * sin_val;
	me->Pdy = (A / 2.0f) * sinf(2.0f * phase);
//	kinematics.target.theta = atan2f(cos_2val, cos_val);
}

static inline void generate_path_trajectory(mw_kinematics_data_t *me, float t)
{
	if (path_handle.total_points == 0 ||
		path_handle.current_index >= path_handle.total_points)
	{
		*(path_handle.is_run)			= false;
		return;
	}

	point_t target = path_handle.points[path_handle.current_index];

	float dx = me->Pdx - me->Px;
	float dy = me->Pdy - me->Py;
	float dist_to_goal = sqrtf(dx * dx + dy * dy);

	float acceptance_radius = 0.2f;

	if (dist_to_goal < acceptance_radius) {
		// Logic chuyển điểm
		if (path_handle.direction == 1) {
			// --- ĐANG ĐI XUÔI ---
			if (path_handle.current_index < path_handle.total_points - 1)
			{
				path_handle.current_index++;
			}
			else
			{
				// Đã đến điểm cuối cùng
				if (path_handle.is_cycle)
				{
					path_handle.direction = -1; // Đảo chiều
				}
				else
				{
					*(path_handle.is_run) = false;
					return;
				}
			}
		}
		else
		{
			// --- ĐANG ĐI NGƯỢC (Hoặc đang Return Home) ---
			if (path_handle.current_index > 0)
			{
				path_handle.current_index--; // Giảm index lùi về 0
			}
			else
			{
				path_handle.direction = 1;
			}
		}
	}

	kinematics.target.theta 	= atan2f(dy, dx);
	kinematics.target.velocity 	= path_handle.velocity;
	kinematics.e_pos			= dist_to_goal;
	kinematics.e_theta			= kinematics.target.theta - kinematics.actual.theta;

	me->Pdx 	= target.x;
	me->Pdy 	= target.y;
	me->Ex 		= dx;
	me->Ey 		= dy;
	me->Etheta	= kinematics.e_theta;
}

static inline void calculate_error(mw_kinematics_data_t *me)
{
	float dx = me->Pdx - me->Px;
	float dy = me->Pdy - me->Py;

	kinematics.e_pos = sqrtf(dx * dx + dy * dy);
	#if !USE_FEEDBACK_LINEARIZATION
		kinematics.target.theta = atan2f(dy, dx);
	#endif
	kinematics.e_theta = kinematics.target.theta - kinematics.actual.theta;
	normalize_angle(&kinematics.e_theta);

	me->Ex 		= dx;
	me->Ey 		= dy;
	me->Etheta 	= kinematics.e_theta;

	calculate_set_point(me);
}

static inline void calculate_set_point(mw_kinematics_data_t *me)
{
	#if (USE_FEEDBACK_LINEARIZATION)
		float theta 		= kinematics.target.theta;
		float x1_curr 		= me->Px + LOOK_AHEAD_DIST * cosf(theta);
		float y1_curr 		= me->Py + LOOK_AHEAD_DIST * sinf(theta);
		float x1_des 		= me->Pdx;
		float y1_des 		= me->Pdy;
		float u1_feedback 	= MW_KINEMATICS_KV * (x1_des - x1_curr);
		float u2_feedback 	= MW_KINEMATICS_KV * (y1_des - y1_curr);
		float u1_ff 		= 0.0f;
		float u2_ff 		= 0.0f;

//		if (trajectory_params.shape == MW_KINEMATICS_PATH &&
//			pRobot->pathCurrentIdx < pRobot->pathTotalPoints - 1)
//		{
//			float v_target = pRobot->shapeParam.vel;
//			if (v_target < 0.1f) v_target = 0.5f; // Giá trị mặc định hợp lý hơn 3.5
//
//			float dx = x1_des - x1_curr;
//			float dy = y1_des - y1_curr;
//			float dist = sqrtf(dx*dx + dy*dy);
//
//			if (dist > 0.001f)
//			{
//				u1_ff = v_target * (dx / dist);
//				u2_ff = v_target * (dy / dist);
//			}
//		}

		float u1 = u1_ff + u1_feedback;
		float u2 = u2_ff + u2_feedback;
		float cos_t = cosf(theta);
		float sin_t = sinf(theta);
		float v_des = u1 * cos_t + u2 * sin_t;
		float w_des = (-u1 * sin_t + u2 * cos_t) / LOOK_AHEAD_DIST;
		float user_vel = trajectory_handle.velocity;
		if (v_des > user_vel)
		{
			float scale = user_vel / v_des;
			v_des *= scale;
			w_des *= scale;
		}

		kinematics.target.velocity 	= v_des;
		kinematics.target.omega 	= w_des;

	#else
		float v_des = MW_KINEMATICS_KV * kinematics.e_pos;
		float w_des = MW_KINEMATICS_KW * kinematics.e_theta;


		float w_R_req = (v_des + (w_des * MW_KINEMATIC_WHEEL_BASE / 2.0f)) / MW_KINEMATIC_WHEEL_RADIUS;
		float w_L_req = (v_des - (w_des * MW_KINEMATIC_WHEEL_BASE / 2.0f)) / MW_KINEMATIC_WHEEL_RADIUS;
		float max_wheel_speed_req = fmaxf(fabsf(w_R_req), fabsf(w_L_req));
		float max_allowed_speed = MOTOR_MAX_OMEGA * MW_KINEMATICS_SAFETY_FACTOR;

		if (max_wheel_speed_req > max_allowed_speed)
		{
			float scale_factor = max_allowed_speed / max_wheel_speed_req;
			kinematics.target.velocity 	= v_des * scale_factor;
			kinematics.target.omega 	= w_des * scale_factor;
		}
		else
		{
			kinematics.target.velocity 	= v_des;
			kinematics.target.omega 	= w_des;
		}
		if(kinematics.target.velocity < 0.0f) kinematics.target.velocity = 0.0f;

	#endif
}

static inline void normalize_angle(float *p_angle)
{
    *p_angle = fmodf(*p_angle, 2.0f * MW_KINEMATICS_PI);

    if (*p_angle > MW_KINEMATICS_PI)
    {
        *p_angle -= 2.0f * MW_KINEMATICS_PI;
    }
    else if (*p_angle < -MW_KINEMATICS_PI)
    {
        *p_angle += 2.0f * MW_KINEMATICS_PI;
    }
}

/* End of file -------------------------------------------------------- */
