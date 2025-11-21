/*
 * nut_communication.c
 *
 *  Created on: Jun 24, 2025
 *      Author: abcde
 */

#include "NutsLib.h"
#include "user.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "string.h"
#include "unistd.h"

static const char *TAG = "Nut-ESP32";

/* Command */
extern NutAction_t command_list[];
extern uint16_t command_count;

/* Command program to be executed */
NutStatus_e (*cmd_program)(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
						   uint32_t result_buffer_MAX_size);

/* Current status of Nut */
NutStatus_e status;
NutError_e error;

/* Buffers and variables for communication */
WORD_ALIGNED_ATTR uint8_t tx_buffer[NUT_BUFFER_SIZE];
WORD_ALIGNED_ATTR uint8_t rx_buffer[NUT_BUFFER_SIZE];
spi_slave_transaction_t hspi;
QueueHandle_t uart_queue;

/* DEBUG */
uint8_t success_cmd_count = 0;
uint8_t received_any_count = 0;

/* Decode header */
uint32_t _NutComm_DecodeHeader()
{
	uint32_t length;
	uint8_t i;
	uint16_t command;
	/* Decode the header */
	command = rx_buffer[0];
	command <<= 8;
	command |= rx_buffer[1];
	length = rx_buffer[4];
	length <<= 8;
	length |= rx_buffer[5];
	length <<= 8;
	length |= rx_buffer[6];
	length <<= 8;
	length |= rx_buffer[7];
	/* Parse command */
	cmd_program = 0;
	for (i = 0; i < command_count; i++)
	{
		if (command_list[i].command == command)
		{
			cmd_program = command_list[i].function; // set the program
			break;
		}
	}
	/* If the command is not found, then continue to receive the payload if possible */
	if (!cmd_program)
	{
		status = NUT_WARNING;
		error = NUT_ERROR_CMD_UNKNOWN;
		ESP_LOGW(TAG, "  Command unknown: %04X", command);
	}
	if (length > NUT_BUFFER_SIZE)
	{
		status = NUT_ERROR;
		error = NUT_ERROR_PAYLOAD_SIZE;
		ESP_LOGE(TAG, "  Input payload size too large: %ld", length);
	}
	return length;
}

/* SPI callbacks */
// Called after a transaction is queued and ready for pickup by master.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
	// gpio_set_level(GPIO_HANDSHAKE, 1);
}

// Called after transaction is sent/received.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
}

/*  INTERFACE INITIALIZATION  */
void _NutComm_UART_Init()
{
	ESP_LOGI(TAG, "  Initializing UART...");
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT};

	uart_param_config(NUT_UART, &uart_config);
	uart_set_pin(NUT_UART, NUT_UART_TX_PIN, NUT_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(NUT_UART, NUT_BUFFER_SIZE, NUT_BUFFER_SIZE, 10, &uart_queue, 0);
}
void _NutComm_SPI_Init()
{
	ESP_LOGI(TAG, "  Initializing SPI...");
	esp_err_t retstatus = ESP_OK;

	// Configuration for the SPI bus
	spi_bus_config_t buscfg = {
		.mosi_io_num = NUT_SPI_MOSI_PIN,
		.miso_io_num = NUT_SPI_MISO_PIN,
		.sclk_io_num = NUT_SPI_SCK_PIN,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
	};

	// Configuration for the SPI slave interface
	spi_slave_interface_config_t slvcfg = {
		.mode = 0,
		.spics_io_num = NUT_SPI_CS_PIN,
		.queue_size = 1,
		.flags = 0,
		.post_setup_cb = my_post_setup_cb,
		.post_trans_cb = my_post_trans_cb};

	// Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
	gpio_set_pull_mode(NUT_SPI_MOSI_PIN, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(NUT_SPI_SCK_PIN, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(NUT_SPI_CS_PIN, GPIO_PULLUP_ONLY);

	// Initialize SPI slave interface
	retstatus = spi_slave_initialize(NUT_SPI, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
	if (retstatus != ESP_OK)
		while (1)
			;

	memset(tx_buffer, 0, NUT_BUFFER_SIZE);
	memset(rx_buffer, 0, NUT_BUFFER_SIZE);
	memset(&hspi, 0, sizeof(hspi));
	hspi.tx_buffer = tx_buffer;
	hspi.rx_buffer = rx_buffer;
	hspi.user = 0; // yes, I am assigning a value to a pointer
}
void _NutComm_I2C_Init()
{
}
void _NutComm_CAN_Init()
{
}
void _NutComm_Pins_Init()
{
	ESP_LOGI(TAG, "  Initializing Pins...");
	gpio_config_t init;

	init.intr_type = GPIO_INTR_DISABLE;		   // 失能中断;
	init.mode = GPIO_MODE_OUTPUT;			   // 输出模式
	init.pin_bit_mask = (1ULL << NUT_IO1_PIN); // IO1
	init.pull_down_en = GPIO_PULLDOWN_DISABLE; // 失能下拉模式
	init.pull_up_en = GPIO_PULLUP_DISABLE;	   // 使能上拉模式
	gpio_config(&init);

	init.intr_type = GPIO_INTR_DISABLE;		   // 失能中断;
	init.mode = GPIO_MODE_OUTPUT;			   // 输出模式
	init.pin_bit_mask = (1ULL << NUT_IO2_PIN); // IO2
	init.pull_down_en = GPIO_PULLDOWN_DISABLE; // 失能下拉模式
	init.pull_up_en = GPIO_PULLUP_DISABLE;	   // 使能上拉模式
	gpio_config(&init);

	init.intr_type = GPIO_INTR_DISABLE;		   // 失能中断;
	init.mode = GPIO_MODE_OUTPUT;			   // 输出模式
	init.pin_bit_mask = (1ULL << NUT_IO3_PIN); // IO3
	init.pull_down_en = GPIO_PULLDOWN_DISABLE; // 失能下拉模式
	init.pull_up_en = GPIO_PULLUP_DISABLE;	   // 使能上拉模式
	gpio_config(&init);

	init.intr_type = GPIO_INTR_DISABLE;			   // 失能中断;
	init.mode = GPIO_MODE_OUTPUT;				   // 输出模式
	init.pin_bit_mask = (1ULL << NUT_IO_USER_PIN); // IO_USER
	init.pull_down_en = GPIO_PULLDOWN_DISABLE;	   // 失能下拉模式
	init.pull_up_en = GPIO_PULLUP_DISABLE;		   // 使能上拉模式
	gpio_config(&init);

	init.intr_type = GPIO_INTR_DISABLE;		   // 失能中断;
	init.mode = GPIO_MODE_OUTPUT;			   // 输出模式
	init.pin_bit_mask = (1ULL << NUT_LED_PIN); // LED
	init.pull_down_en = GPIO_PULLDOWN_DISABLE; // 失能下拉模式
	init.pull_up_en = GPIO_PULLUP_DISABLE;	   // 使能上拉模式
	gpio_config(&init);
}

/* Last things to do before next communucation */
void _NutComm_UART_Quit()
{
	//	HAL_UART_Abort(&NUT_UART);
	//	NUT_UART.RxState = HAL_UART_STATE_READY;
	//	NUT_UART.gState = HAL_UART_STATE_READY;
}
void _NutComm_SPI_Quit()
{
	/* Wait until CSn releases */
	while (gpio_get_level(NUT_SPI_CS_PIN) == 0)
		;
}
void _NutComm_I2C_Quit()
{
}

/* Error handlers, in case they are needed */
void _NutComm_SPI_Error()
{
	/* Wait until chip not selected */
	while (gpio_get_level(NUT_SPI_CS_PIN) == 0)
		;
}
void _NutComm_I2C_Error()
{
	i2c_reset_rx_fifo(NUT_I2C);
	i2c_reset_tx_fifo(NUT_I2C);
}
void _NutComm_CAN_Error()
{
	// CAN not supported on ESP32
}

/* Initializes communication interfaces */
void _NutComm_Init()
{
	_NutComm_Pins_Init();
	_NutComm_UART_Init();
	_NutComm_SPI_Init();
	_NutComm_I2C_Init();
	_NutComm_CAN_Init();
}

/* Continuously check for signs of communication */
void Nut_Loop()
{
	esp_err_t retstatus = ESP_OK;
	size_t length;
	uint32_t response_length = 0;
	uint32_t i;

	/* Check UART */
	uart_get_buffered_data_len(NUT_UART, &length);
	if (length > 0)
	{
		/* Record the first byte of header */
		retstatus = uart_read_bytes(NUT_UART, rx_buffer, 1, 10);

		/* Finish receiving the header */
		for (i = 1; i < 8;)
		{
			retstatus = uart_read_bytes(NUT_UART, rx_buffer + i, 1, 10);
			if (retstatus == 1)
			{
				i++;
			}
			/* in case of timeout and error, must exit because UART is special and must not get stuck */
			else if (retstatus == -1)
			{
				status = NUT_ERROR;
				error = NUT_ERROR_UNKNOWN;
				ESP_LOGE(TAG, "  Timeout or error during UART header receive.[1]");
				_NutComm_UART_Quit();
				return;
			}
		}

		/* Decode header */
		status = NUT_OK;
		length = _NutComm_DecodeHeader();
		/* If ERROR occurred, prepare to send error package */
		if (status == NUT_ERROR)
		{
			/* Wait until the other side to finish transmission */
			for (i = 0; i < length;)
			{
				retstatus = uart_read_bytes(NUT_UART, rx_buffer + i, 1, 10);
				if (retstatus == 1)
				{
					i++;
				}
				/* in case of timeout and error, must exit because UART is special and must not get stuck */
				else if (retstatus == -1)
				{
					status = NUT_ERROR;
					error = NUT_ERROR_UNKNOWN;
					ESP_LOGE(TAG, "  Timeout or error during UART header receive.[2]");
					_NutComm_UART_Quit();
					return;
				}
			}
			/* Send error package */
			tx_buffer[0] = NUT_ERROR;
			tx_buffer[1] = NUT_ERROR_PAYLOAD_SIZE;
			tx_buffer[2] = 0;
			tx_buffer[3] = 0;
			tx_buffer[4] = 0;
			tx_buffer[5] = 0;
			uart_write_bytes(NUT_UART, tx_buffer, 6); // no need to check whether it is successful
			_NutComm_UART_Quit();
			return;
		}
		/* Receive the payload */
		for (i = 0; i < length;)
		{
			retstatus = uart_read_bytes(NUT_UART, rx_buffer + i, 1, 10);
			if (retstatus == 1)
			{
				i++;
			}
			/* in case of timeout and error, must exit because UART is special and must not get stuck */
			else if (retstatus == -1)
			{
				status = NUT_ERROR;
				error = NUT_ERROR_UNKNOWN;
				ESP_LOGE(TAG, "  Timeout or error during UART header receive.[3]");
				_NutComm_UART_Quit();
				return;
			}
		}
		/* Process the command and give feedback */
		if (cmd_program)
		{
			status = cmd_program(rx_buffer, length, tx_buffer, &response_length, NUT_BUFFER_SIZE);
			/* Send response package according to response length */
			if (response_length > NUT_BUFFER_SIZE)
			{
				tx_buffer[0] = NUT_ERROR;
				tx_buffer[1] = NUT_ERROR_PAYLOAD_SIZE;
				tx_buffer[2] = 0;
				tx_buffer[3] = 0;
				tx_buffer[4] = 0;
				tx_buffer[5] = 0;
				uart_write_bytes(NUT_UART, tx_buffer, 6); // no need to check whether it is successful
				ESP_LOGE(TAG, "  Output payload size too large: %ld", response_length);
				_NutComm_UART_Quit();
				return;
			}
			else
			{
				/* Prepare header */
				if (status == NUT_OK)
				{
					tx_buffer[0] = NUT_OK;
					tx_buffer[1] = 0;
				}
				else
				{
					tx_buffer[0] = NUT_ERROR;
					tx_buffer[1] = NUT_ERROR_USER_CODE;
				}
				tx_buffer[2] = 0x0FF & (response_length >> 24);
				tx_buffer[3] = 0x0FF & (response_length >> 16);
				tx_buffer[4] = 0x0FF & (response_length >> 8);
				tx_buffer[5] = 0x0FF & (response_length);
				/* Send the header */
				uart_write_bytes(NUT_UART, tx_buffer, 6);
				/* Then send the payload */
				for (i = 0; i < response_length;)
				{
					retstatus = uart_write_bytes(NUT_UART, tx_buffer + i, 1);
					if (retstatus == 1)
					{
						i++;
					}
					/* in case of timeout and error, must exit because UART is special and must not get stuck */
					else if (retstatus == -1)
					{
						status = NUT_ERROR;
						error = NUT_ERROR_UNKNOWN;
						ESP_LOGE(TAG, "  Timeout or error during UART header receive.[3]");
						_NutComm_UART_Quit();
						return;
					}
				}
				_NutComm_UART_Quit();
				return;
			}
		}
		/* Command not found, return error */
		else
		{
			tx_buffer[0] = NUT_ERROR;
			tx_buffer[1] = NUT_ERROR_CMD_UNKNOWN;
			tx_buffer[2] = 0;
			tx_buffer[3] = 0;
			tx_buffer[4] = 0;
			tx_buffer[5] = 0;
			uart_write_bytes(NUT_UART, tx_buffer, 6); // no need to check whether it is successful
			ESP_LOGE(TAG, "  Command not found.");
			_NutComm_UART_Quit();
			return;
		}
	}

	/* Check SPI */
	else if (gpio_get_level(NUT_SPI_CS_PIN) == 0)
	{
		/* Poll for header */
		hspi.length = 8;
		retstatus = spi_slave_transmit(NUT_SPI, &hspi, 100);
		if (retstatus != ESP_OK)
		{
			status = NUT_WARNING;
			error = NUT_ERROR_SPI_ABORTED;
			_NutComm_SPI_Quit();
			return;
		}
		/* if CS is released midway (in case of timeout) */
		if (gpio_get_level(NUT_SPI_CS_PIN) != 0)
		{
			status = NUT_WARNING;
			error = NUT_ERROR_SPI_ABORTED;
			_NutComm_SPI_Quit();
			return;
		}
		/* Decode header */
		length = _NutComm_DecodeHeader();
		/* If ERROR occurred, prepare to send error package */
		if (status == NUT_ERROR)
		{
			/* Wait until the other side to finish transmission */
			for (i = 0; i < length;)
			{
				hspi.length = 1;
				retstatus = spi_slave_transmit(NUT_SPI, &hspi, 100);
				if (retstatus == ESP_OK)
				{
					i++;
				}
				/* if CS is released midway (in case of timeout) */
				else if (gpio_get_level(NUT_SPI_CS_PIN) != 0)
				{
					status = NUT_WARNING;
					error = NUT_ERROR_SPI_ABORTED;
					_NutComm_SPI_Quit();
					return;
				}
			}
			/* Send error package */
			tx_buffer[0] = NUT_ERROR;
			tx_buffer[1] = NUT_ERROR_PAYLOAD_SIZE;
			tx_buffer[2] = 0;
			tx_buffer[3] = 0;
			tx_buffer[4] = 0;
			tx_buffer[5] = 0;
			hspi.length = 6;
			retstatus = spi_slave_transmit(NUT_SPI, &hspi, 100);
			if (retstatus != ESP_OK)
			{
				status = NUT_WARNING;
				error = NUT_ERROR_SPI_ABORTED;
				_NutComm_SPI_Quit();
				return;
			}
			/* if CS is released midway (in case of timeout) */
			if (gpio_get_level(NUT_SPI_CS_PIN) != 0)
			{
				status = NUT_WARNING;
				error = NUT_ERROR_SPI_ABORTED;
				_NutComm_SPI_Quit();
				return;
			}
			/* And just return */
			_NutComm_SPI_Quit();
			return;
		}
		/* Receive the payload */
		hspi.length = length;
		hspi.user = 0; // yes, I am assigning a value to a pointer
		retstatus = spi_slave_transmit(NUT_SPI, &hspi, 100);
		if (retstatus != ESP_OK)
		{
			status = NUT_WARNING;
			error = NUT_ERROR_SPI_ABORTED;
			_NutComm_SPI_Quit();
			return;
		}
		/* Process the command and give feedback */
		if (cmd_program)
		{
			status = cmd_program(rx_buffer, length, tx_buffer, &response_length, NUT_BUFFER_SIZE);
			/* Send response package according to response length */
			if (response_length > NUT_BUFFER_SIZE)
			{
				tx_buffer[0] = NUT_ERROR;
				tx_buffer[1] = NUT_ERROR_PAYLOAD_SIZE;
				tx_buffer[2] = 0;
				tx_buffer[3] = 0;
				tx_buffer[4] = 0;
				tx_buffer[5] = 0;
				hspi.length = 6;
				retstatus = spi_slave_transmit(NUT_SPI, &hspi, 100);
				if (retstatus != ESP_OK)
				{
					status = NUT_WARNING;
					error = NUT_ERROR_SPI_ABORTED;
					_NutComm_SPI_Quit();
					return;
				}
				/* if CS is released midway (in case of timeout) */
				if (gpio_get_level(NUT_SPI_CS_PIN) != 0)
				{
					status = NUT_WARNING;
					error = NUT_ERROR_SPI_ABORTED;
					_NutComm_SPI_Quit();
					return;
				}
				_NutComm_SPI_Quit();
				return;
			}
			else
			{
				/* Prepare header */
				if (status == NUT_OK)
				{
					tx_buffer[0] = NUT_OK;
					tx_buffer[1] = 0;
				}
				else
				{
					tx_buffer[0] = NUT_ERROR;
					tx_buffer[1] = NUT_ERROR_USER_CODE;
				}
				tx_buffer[2] = 0x0FF & (response_length >> 24);
				tx_buffer[3] = 0x0FF & (response_length >> 16);
				tx_buffer[4] = 0x0FF & (response_length >> 8);
				tx_buffer[5] = 0x0FF & (response_length);
				/* Send the header */
				hspi.length = 6;
				retstatus = spi_slave_transmit(NUT_SPI, &hspi, 100);
				if (retstatus != ESP_OK)
				{
					status = NUT_WARNING;
					error = NUT_ERROR_SPI_ABORTED;
					_NutComm_SPI_Quit();
					return;
				}
				/* if CS is released midway (in case of timeout) */
				if (gpio_get_level(NUT_SPI_CS_PIN) != 0)
				{
					status = NUT_WARNING;
					error = NUT_ERROR_SPI_ABORTED;
					_NutComm_SPI_Quit();
					return;
				}
				/* Then send the payload */
				hspi.length = response_length;
				retstatus = spi_slave_transmit(NUT_SPI, &hspi, 100);
				if (retstatus != ESP_OK)
				{
					status = NUT_WARNING;
					error = NUT_ERROR_SPI_ABORTED;
					_NutComm_SPI_Quit();
					return;
				}
				/* if CS is released midway (in case of timeout) */
				if (gpio_get_level(NUT_SPI_CS_PIN) != 0)
				{
					status = NUT_WARNING;
					error = NUT_ERROR_SPI_ABORTED;
					_NutComm_SPI_Quit();
					return;
				}
				_NutComm_SPI_Quit();
				return;
			}
		}
		/* Command not found, return error */
		else
		{
			tx_buffer[0] = NUT_ERROR;
			tx_buffer[1] = NUT_ERROR_CMD_UNKNOWN;
			tx_buffer[2] = 0;
			tx_buffer[3] = 0;
			tx_buffer[4] = 0;
			tx_buffer[5] = 0;
			hspi.length = 6;
			retstatus = spi_slave_transmit(NUT_SPI, &hspi, 100);
			if (retstatus != ESP_OK)
			{
				status = NUT_WARNING;
				error = NUT_ERROR_SPI_ABORTED;
				_NutComm_SPI_Quit();
				return;
			}
			/* if CS is released midway (in case of timeout) */
			if (gpio_get_level(NUT_SPI_CS_PIN) != 0)
			{
				status = NUT_WARNING;
				error = NUT_ERROR_SPI_ABORTED;
				_NutComm_SPI_Quit();
				return;
			}
			_NutComm_SPI_Quit();
			return;
		}
	}

	/* Check I2C */
	else if (0)
	{
		// TODO
	}
}

void Nut_Init()
{
	ESP_LOGI(TAG, "Nut Initialization Start.");
	_NutComm_Init();
	User_Init();
	ESP_LOGI(TAG, "Nut Initialization Done.");
}

void Nut_Quiet()
{
	// SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // disable SysTick
}

void Nut_Unquiet()
{
	// SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // enable SysTick
}

/* Pin Manipulation */

void Nut_LED(uint8_t on)
{
	gpio_set_level(NUT_LED_PIN, on ? 0 : 1);
}

void Nut_IO_1(uint8_t set)
{
	gpio_set_level(NUT_IO1_PIN, set ? 1 : 0);
}

void Nut_IO_2(uint8_t set)
{
	gpio_set_level(NUT_IO2_PIN, set ? 1 : 0);
}

void Nut_IO_3(uint8_t set)
{
	gpio_set_level(NUT_IO3_PIN, set ? 1 : 0);
}

void Nut_Trigger_Set(void)
{
	Nut_IO_1(1);
}

void Nut_Trigger_Clear(void)
{
	Nut_IO_1(0);
}

uint8_t Nut_IO_USER()
{
	return (uint8_t)gpio_get_level(NUT_IO_USER_PIN);
}
