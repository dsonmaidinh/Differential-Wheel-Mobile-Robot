/**
 * @file       bsp_esp.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-12
 * @author     Dong Son
 *
 * @brief
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef INC_BSP_ESP_H_
#define INC_BSP_ESP_H_

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sys_config.h"

/* Public defines ----------------------------------------------------- */
#define BSP_ESP_MAX_BUFFER 		4200
#define BSP_ESP_PAYLOAD_SIZE 	sizeof(stream_tx_data_t)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
	BSP_ESP_OK = 0,    /**< Operation successful */
	BSP_ESP_ERR_PARAM, /**< Invalid argument */
	BSP_ESP_ERR_COMM,  /**< Communication failure */
	BSP_ESP_ERR        /**< General error */
} bsp_esp_err_t;

typedef enum
{
	BSP_ESP_CIRCLE = 0,
	BSP_ESP_SQUARE,
	BSP_ESP_TRIANGLE,
	BSP_ESP_FIGURE8,
	BSP_ESP_PATH,
	BSP_ESP_LINE,
} bsp_esp_shape_t;

typedef enum
{
	BSP_ESP_WIFI_CONNECTING = 0x00,
	BSP_ESP_WIFI_CONNECTED  = 0x01,
} bsp_esp_wifi_status_t;

typedef enum
{
	BSP_ESP_BATTERY_HIGH = 0x00,
	BSP_ESP_BATTERY_MID  = 0x01,
	BSP_ESP_BATTERY_LOW  = 0x02,
} bsp_esp_battery_status_t;

typedef enum
{
	BSP_ESP_CMD_STOP = 0,
	BSP_ESP_CMD_RUN,
	BSP_ESP_CMD_RETURN_HOME,
	BSP_ESP_CMD_CONFIG,
	BSP_ESP_CMD_ERR,
	BSP_ESP_CMD_NONE,
} bsp_esp_state_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern uint8_t esp_rx_bufer[BSP_ESP_MAX_BUFFER];

/* Public function prototypes ----------------------------------------- */
bsp_esp_err_t bsp_esp_init(uint8_t *wifi_status, uint8_t *battery_status);

bsp_esp_state_t bsp_esp_get_data();

bsp_esp_err_t bsp_esp_send_data(uint8_t *data);

#endif /* INC_BSP_ESP_H_ */

/* End of file -------------------------------------------------------- */
