/**
 * @file       bsp_esp.c
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-02-12
 * @author     Dong Son
 *
 * @brief
 */
/* Includes ----------------------------------------------------------- */
#include <bsp_esp.h>
#include "err.h"
#include "stm32f1xx.h"
#include <string.h>
#include <stddef.h>

/* Private defines ---------------------------------------------------- */
#define BSP_ESP_UART_HANDLE 		huart5
#define BSP_ESP_USART				UART5
#define BSP_ESP_CMD_HEADER_BYTE1    0xAA
#define BSP_ESP_CMD_HEADER_BYTE2    0xBB
#define BSP_ESP_BATCH_HEADER_BYTE1  0xAA
#define BSP_ESP_BATCH_HEADER_BYTE2  0x55
#define BSP_ESP_BATCH_FOOTER		0xEE
#define BSP_ESP_BATCH_SIZE 			4
#define BSP_ESP_SEND_TIME_MS  		40
#define BSP_ESP_SAMPLE_TIME_MS 		BSP_ESP_SEND_TIME_MS/BSP_ESP_BATCH_SIZE
#define BSP_ESP_SAMPLE_COUNT 		BSP_ESP_SAMPLE_TIME_MS/BSP_ESP_SAMPLE_TIME_MS
#define BSP_ESP_MIN_PACKET_SIZE 	3

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
    CMD_STOP,
    CMD_RUN,
    CMD_RETURN_HOME,
    CMD_SET_TRAJECTORY,
	CMD_SET_PATH,
	CMD_STATUS,
	CMD_NONE
} bsp_esp_cmd_t;

typedef struct
{
	bsp_esp_wifi_status_t 		*wifi_status;
	bsp_esp_battery_status_t	*battery_status;
} bsp_esp_status_t;

typedef stream_rx_status_t bsp_esp_rx_status_t;

#pragma pack(1)
typedef struct {
    uint8_t 			header[2];
    uint8_t 			samples[BSP_ESP_BATCH_SIZE][BSP_ESP_PAYLOAD_SIZE];
    uint8_t 			footer;
} bsp_esp_tx_batch_data_t;
#pragma pack()

typedef struct {
    bsp_esp_tx_batch_data_t batch_buffers[2];
    uint8_t active_buf_idx;
    uint8_t sample_idx;
} bsp_esp_tx_batch_t;
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern UART_HandleTypeDef BSP_ESP_UART_HANDLE;
uint8_t esp_rx_bufer[BSP_ESP_MAX_BUFFER];

/* Private variables -------------------------------------------------- */
static bool 			is_received 	= false;
static bool 			is_send_busy 	= false;
static bool 			is_err			= false;
static uint16_t 		tx_size			= 0;
static uint16_t 		rx_size			= 0;
bsp_esp_tx_batch_t		esp_tx_buffer;
bsp_esp_status_t		status;
/* Private function prototypes ---------------------------------------- */
static inline void bsp_esp_set_status();

static inline void bsp_esp_data_validate();

static inline uint8_t bsp_esp_calculate_checksum();

/* Function definitions ----------------------------------------------- */
bsp_esp_err_t bsp_esp_init(uint8_t *wifi_status, uint8_t *battery_status)
{
	CHECK_PARAM((wifi_status == NULL) || (battery_status == NULL), BSP_ESP_ERR_PARAM);
	status.wifi_status		= wifi_status;
	status.battery_status 	= battery_status;

	tx_size = sizeof(bsp_esp_tx_batch_data_t);

	esp_tx_buffer.active_buf_idx = 0;
	esp_tx_buffer.sample_idx = 0;
	for (int i = 0; i < 2; ++i)
	{
		esp_tx_buffer.batch_buffers[i].header[0] 	= BSP_ESP_BATCH_HEADER_BYTE1;
		esp_tx_buffer.batch_buffers[i].header[1] 	= BSP_ESP_BATCH_HEADER_BYTE2;
		esp_tx_buffer.batch_buffers[i].footer 		= BSP_ESP_BATCH_FOOTER;
	}

	HAL_UARTEx_ReceiveToIdle_IT(&BSP_ESP_UART_HANDLE, esp_rx_bufer, BSP_ESP_MAX_BUFFER);

	return BSP_ESP_OK;
}

bsp_esp_state_t bsp_esp_get_data()
{
	if(is_err)
	{
		is_err = false;
		is_received = false;
		return BSP_ESP_CMD_ERR;
	}

	if(!is_received) return BSP_ESP_CMD_NONE;
	is_received = false;

	uint8_t cmd = esp_rx_bufer[2];

	switch(cmd)
	{
		case STREAM_CMD_STATUS:
			bsp_esp_set_status();
			return BSP_ESP_CMD_NONE;
		case STREAM_CMD_STOP:
			return BSP_ESP_CMD_STOP;
		case STREAM_CMD_RUN:
			return BSP_ESP_CMD_RUN;
		case STREAM_CMD_SET_TRAJECTORY:
		case STREAM_CMD_SET_PATH:
			return BSP_ESP_CMD_CONFIG;
		case STREAM_RETURN_HOME:
			return BSP_ESP_CMD_RETURN_HOME;
		default:
			return BSP_ESP_CMD_NONE;
	}
}

bsp_esp_err_t bsp_esp_send_data(uint8_t *data)
{
	memcpy(esp_tx_buffer.batch_buffers[esp_tx_buffer.active_buf_idx].samples[esp_tx_buffer.sample_idx],
				data,
		        BSP_ESP_PAYLOAD_SIZE);

	esp_tx_buffer.sample_idx++;
	if(esp_tx_buffer.sample_idx >= BSP_ESP_BATCH_SIZE)
	{
		esp_tx_buffer.sample_idx 		= 0;
		if(is_send_busy)
		{
//			is_err = true;
			return BSP_ESP_ERR_COMM;
		}

		bsp_esp_tx_batch_data_t *p_batch = &esp_tx_buffer.batch_buffers[esp_tx_buffer.active_buf_idx];
		is_send_busy = true;
		HAL_UART_Transmit_IT(&BSP_ESP_UART_HANDLE, (uint8_t *)p_batch, tx_size);
		esp_tx_buffer.active_buf_idx 	= !esp_tx_buffer.active_buf_idx;
	}

	return BSP_ESP_OK;
}

/* Private definitions ------------------------------------------------ */
static inline void bsp_esp_set_status()
{
	if(esp_rx_bufer[2] == CMD_STATUS)
	{
		bsp_esp_rx_status_t *p_status = (bsp_esp_rx_status_t *)esp_rx_bufer;
		*(status.wifi_status) 		= p_status->wifi_status;
		*(status.battery_status) 	= p_status->battery_status;
	}
}

static inline void bsp_esp_data_validate()
{
	if (rx_size < BSP_ESP_MIN_PACKET_SIZE)
	{
		is_err = true;
		return;
	}

	if((esp_rx_bufer[0] != BSP_ESP_CMD_HEADER_BYTE1 || esp_rx_bufer[1] != BSP_ESP_CMD_HEADER_BYTE2))
	{
		is_err = true;
		return;
	}

	uint8_t received_sum = esp_rx_bufer[rx_size - 1];
	uint8_t calc_sum = bsp_esp_calculate_checksum();

	if (received_sum != calc_sum)
	{
		is_err = true;
		return;
	}
}

static inline uint8_t bsp_esp_calculate_checksum()
{
	uint8_t sum = 0;
	for (uint16_t i = 0; i < rx_size - 1; i++) {
		sum += esp_rx_bufer[i];
	}
	return sum;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == BSP_ESP_USART)
    {
    	is_received = true;
    	rx_size = Size;
    	bsp_esp_data_validate();
    	HAL_UARTEx_ReceiveToIdle_IT(&BSP_ESP_UART_HANDLE, esp_rx_bufer, BSP_ESP_MAX_BUFFER);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == BSP_ESP_USART) {
    	is_send_busy = false;
    }
}

/* End of file -------------------------------------------------------- */
