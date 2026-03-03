/**
 * @file       bsp_led.c
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-14
 * @author     Dong Son
 *
 * @brief
 */
/* Includes ----------------------------------------------------------- */
#include "bsp_led.h"
#include <stdbool.h>
#include "stm32f1xx.h"

/* Private defines ---------------------------------------------------- */
#define BSP_LED_BLINK_SLOW		500u
#define BSP_LED_BLINK_FAST		200u
#define BSP_LED_NO_BLINK		0u
#define BSP_LED_COUNT			2u
#define BSP_LED_ERR_PORT		GPIOA
#define BSP_LED_ERR_PIN			GPIO_PIN_15
#define BSP_LED_WIFI_PORT		GPIOC
#define BSP_LED_WIFI_PIN		GPIO_PIN_11
#define BSP_LED_BATTERY_PORT	GPIOC
#define BSP_LED_BATTERY_PIN		GPIO_PIN_10
#define BSP_LED_TIM_HANDLE 		htim4

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
	BSP_LED_BATTERY_STATUS,
	BSP_LED_WIFI_STATUS,
}bsp_led_index_t;

typedef struct
{
	uint16_t 	interval;
	bool 		is_blinking;
} bsp_led_config_t;

typedef struct
{
	GPIO_TypeDef 		*port;
	uint16_t			pin;
	void 				*status;
	bsp_led_config_t 	*config_map;
	uint32_t			tick;
} bsp_led_io_t;

typedef struct
{
	uint8_t *wifi_status;
	uint8_t *battery_status;
}bsp_led_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern TIM_HandleTypeDef BSP_LED_TIM_HANDLE;

/* Private variables -------------------------------------------------- */
bsp_led_config_t led_wifi_handle[] =
{
	{ BSP_LED_BLINK_SLOW, 	true},
	{ BSP_LED_NO_BLINK, 	false},
};

bsp_led_config_t led_battery_handle[] =
{
	{ BSP_LED_NO_BLINK, 	false},
	{ BSP_LED_BLINK_SLOW, 	true},
	{ BSP_LED_BLINK_FAST, 	true},
};

bsp_led_io_t	led_handle[BSP_LED_COUNT] = {0};

static uint8_t counter = 0;
/* Private function prototypes ---------------------------------------- */
static inline void bsp_led_err_process();

/* Function definitions ----------------------------------------------- */
bsp_led_err_t bsp_led_set_init(uint8_t *wifi_status, uint8_t *battery_status)
{
    // --- MAP LED BATTERY ---
	led_handle[BSP_LED_BATTERY_STATUS].port 		= BSP_LED_BATTERY_PORT;
	led_handle[BSP_LED_BATTERY_STATUS].pin 			= BSP_LED_BATTERY_PIN;
	led_handle[BSP_LED_BATTERY_STATUS].status 		= battery_status;
	led_handle[BSP_LED_BATTERY_STATUS].config_map 	= led_battery_handle;
	led_handle[BSP_LED_BATTERY_STATUS].tick 		= 0;

	// --- MAP LED WIFI ---
	led_handle[BSP_LED_WIFI_STATUS].port 			= BSP_LED_WIFI_PORT;
	led_handle[BSP_LED_WIFI_STATUS].pin 			= BSP_LED_WIFI_PIN;
	led_handle[BSP_LED_WIFI_STATUS].status 			= wifi_status;
	led_handle[BSP_LED_WIFI_STATUS].config_map 		= led_wifi_handle;
	led_handle[BSP_LED_WIFI_STATUS].tick 			= 0;

	for(uint8_t i = 0; i < BSP_LED_COUNT; i++)
	{
		HAL_GPIO_WritePin(led_handle[i].port, led_handle[i].pin, GPIO_PIN_SET);
	}
	HAL_TIM_Base_Start(&BSP_LED_TIM_HANDLE);

	uint16_t current_cnt = __HAL_TIM_GET_COUNTER(&BSP_LED_TIM_HANDLE);
	__HAL_TIM_SET_COMPARE(&BSP_LED_TIM_HANDLE, TIM_CHANNEL_1, current_cnt + 50000);

	HAL_TIM_OC_Start_IT(&BSP_LED_TIM_HANDLE, TIM_CHANNEL_1);
	return BSP_LED_OK;
}

bsp_led_err_t bsp_led_process()
{
	for(uint8_t i = 0; i < BSP_LED_COUNT; i++)
	{
		if(led_handle[i].status == NULL || led_handle[i].config_map == NULL)
		{
			continue;
		}

		uint8_t current_status 	= *(uint8_t*)(led_handle[i].status);
		bool is_blink 			= led_handle[i].config_map[current_status].is_blinking;
		uint16_t blink_interval = led_handle[i].config_map[current_status].interval;

		if(is_blink)
		{
			led_handle[i].tick += 100;
			if( led_handle[i].tick >= blink_interval )
			{
				led_handle[i].tick = 0;
				HAL_GPIO_TogglePin(led_handle[i].port, led_handle[i].pin);
			}
		}
		else
		{
			led_handle[i].tick = 0;
			HAL_GPIO_WritePin(led_handle[i].port, led_handle[i].pin, GPIO_PIN_RESET);
		}
	}

	return BSP_LED_OK;
}

/* Private definitions ------------------------------------------------ */
void System_Error_Hook(void)
{
    bsp_led_err_process();
}

static inline void bsp_led_err_process()
{
	while(1)
	{
		HAL_GPIO_TogglePin(BSP_LED_ERR_PORT, BSP_LED_ERR_PIN);
		HAL_Delay(500);
	}

}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &BSP_LED_TIM_HANDLE)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
           uint16_t current_ccr = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, current_ccr + 50000);

            if(++counter < 2) return;
            counter = 0;
            bsp_led_process();
        }
    }
}

/* End of file -------------------------------------------------------- */
