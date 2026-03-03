/**
 * @file       sys_config.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-09
 * @author     Dong Son
 *
 * @brief
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef INC_SYS_CONFIG_H_
#define INC_SYS_CONFIG_H_

/* Includes ----------------------------------------------------------- */
#include <stdint.h>

/* Public defines ----------------------------------------------------- */
// Kinematics
#define USE_FEEDBACK_LINEARIZATION	0
#define LOOK_AHEAD_DIST				0.05f
#define MAX_PATH_POINTS				500u
#define WHEEL_RADIUS				0.0344f
#define WHEEL_BASE					0.2f
#define KV 							6.0f
#define KW 							10.0f
#define SAFETY_FACTOR 				0.95f
#define PI							3.1415926535f
// Controller
#define PID_MAX_OUTPUT 				1000.0f
// Motor
#define MOTOR_MAX_OMEGA				12.0f
#define MOTOR_ENCODER_PPR			2059u	// x4
#define MOTOR_COUNT 				2

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
	STREAM_CIRCLE = 0,
	STREAM_SQUARE,
	STREAM_TRIANGLE,
	STREAM_FIGURE8,
	STREAM_PATH,
} stream_trajectory_shape_t;

typedef enum
{
	STREAM_WIFI_CONNECTING = 0x00,
	STREAM_WIFI_CONNECTED  = 0x01,
} stream_wifi_status_t;

typedef enum
{
	STREAM_BATTERY_HIGH = 0x00,
	STREAM_BATTERY_MID  = 0x01,
	STREAM_BATTERY_LOW  = 0x02,
} stream_battery_status_t;

typedef enum
{
    STREAM_CMD_STOP = 0,
	STREAM_CMD_RUN,
	STREAM_CMD_RETURN_HOME,
	STREAM_CMD_SET_TRAJECTORY,
	STREAM_CMD_SET_PATH,
	STREAM_CMD_STATUS,
	STREAM_CMD_NONE
} stream_cmd_t;

typedef enum
{
	STREAM_STOP = 0,
	STREAM_RUN,
	STREAM_RETURN_HOME,
	STREAM_CONFIG,
	STREAM_NONE,
} stream_state_t;

#pragma pack(1)
typedef struct
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t shape;
    float 	param;
    float 	velocity;
    uint8_t checksum;
} stream_rx_trajectory_config_t;

typedef struct
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t wifi_status;
    uint8_t battery_status;
    uint8_t checksum;
} stream_rx_status_t;

typedef struct
{
    uint8_t 	header[2];
    uint8_t 	cmd;
    uint16_t 	count;
	float 		velocity;
    uint8_t 	is_cycle;
    float 		start_x;
    float 		start_y;
} stream_rx_path_config_t;

typedef struct
{
	float Pdx;
	float Pdy;
	float Px;
	float Py;
	float V;
	float W;
	float Ex;
	float Ey;
	float Etheta;
} stream_tx_data_t;
#pragma pack()

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#endif /* INC_SYS_CONFIG_H_ */

/* End of file -------------------------------------------------------- */
