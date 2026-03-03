/**
 * @file       bsp_led.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-14
 * @author     Dong Son
 *
 * @brief
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef INC_BSP_STATUS_H_
#define INC_BSP_STATUS_H_

/* Includes ----------------------------------------------------------- */
#include <stdint.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
	BSP_LED_OK,
	BSP_LED_ERR,
	BSP_LED_ERR_PARAM,
} bsp_led_err_t;

typedef enum
{
	BSP_LED_WIFI_CONNECTING = 0x00,
	BSP_LED_WIFI_CONNECTED  = 0x01,
} bsp_led_wifi_status_t;

typedef enum
{
	BSP_LED_BATTERY_HIGH = 0x00,
	BSP_LED_BATTERY_MID  = 0x01,
	BSP_LED_BATTERY_LOW  = 0x02,
} bsp_led_battery_status_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
bsp_led_err_t 	bsp_led_set_init(uint8_t *wifi_status, uint8_t *battery_status);

bsp_led_err_t 	bsp_led_process();

#endif /* INC_BSP_STATUS_H_ */

/* End of file -------------------------------------------------------- */
