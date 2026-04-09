#include "user.h"
#include <stdlib.h>
#include "esp_log.h"
#include "unistd.h"

static const char *TAG = "User-ESP32";

/*		Command Function Template
 *

 NutStatus_e FunctionName(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
 uint32_t result_buffer_MAX_size) {
 *result_length = 0;		// length of responding payload


 // return run result: NUT_OK, NUT_WARNING, NUT_ERROR
 return NUT_OK;
 }

 */

NutStatus_e Echo(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
				 uint32_t result_buffer_MAX_size)
{
	uint32_t i;
	uint8_t tempu8;
	*result_length = received_data_length;
	for (i = 0; i < received_data_length; i++)
	{
		tempu8 = received_data_ptr[i];
		result_buffer_ptr[i] = tempu8;
	}
	return NUT_OK;
}

#include "esp_sleep.h"
#include "driver/gpio.h"
#include <rom/ets_sys.h>

void set_all_pins_pullup()
{
	gpio_set_direction(NUT_UART_TX_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_UART_RX_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_SPI_CS_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_SPI_SCK_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_SPI_MOSI_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_SPI_MISO_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_I2C_SCL_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_I2C_SDA_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_LED_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_IO1_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_IO2_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_IO3_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(NUT_IO_USER_PIN, GPIO_MODE_INPUT);
	gpio_pullup_en(NUT_UART_TX_PIN);
	gpio_pullup_en(NUT_UART_RX_PIN);
	gpio_pullup_en(NUT_SPI_CS_PIN);
	gpio_pullup_en(NUT_SPI_SCK_PIN);
	gpio_pullup_en(NUT_SPI_MOSI_PIN);
	gpio_pullup_en(NUT_SPI_MISO_PIN);
	gpio_pullup_en(NUT_I2C_SCL_PIN);
	gpio_pullup_en(NUT_I2C_SDA_PIN);
	gpio_pullup_en(NUT_LED_PIN);
	gpio_pullup_en(NUT_IO1_PIN);
	gpio_pullup_en(NUT_IO2_PIN);
	gpio_pullup_en(NUT_IO3_PIN);
	gpio_pullup_en(NUT_IO_USER_PIN);
}

NutStatus_e EnterLightSleep_NoWake(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
								   uint32_t result_buffer_MAX_size)
{
	*result_length = 0;
	Nut_LED(1);
	ets_delay_us(100000);
	Nut_LED(0);
	set_all_pins_pullup();
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
	esp_light_sleep_start();
	return NUT_OK;
}

NutStatus_e EnterDeepSleep_NoWake(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
								  uint32_t result_buffer_MAX_size)
{
	*result_length = 0;
	Nut_LED(1);
	ets_delay_us(100000);
	Nut_LED(0);
	ets_delay_us(100000);
	Nut_LED(1);
	ets_delay_us(100000);
	Nut_LED(0);
	ets_delay_us(100000);
	Nut_LED(1);
	ets_delay_us(100000);
	Nut_LED(0);
	set_all_pins_pullup();
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
	esp_deep_sleep_start();
	return NUT_OK;
}

/* User command */
// @formatter:off
NutAction_t command_list[] = {
	{.command = 0x0001, .function = Echo},
	{.command = 0x0501, .function = EnterLightSleep_NoWake},
	{.command = 0x0503, .function = EnterDeepSleep_NoWake},
};
// @formatter:on
uint16_t command_count = sizeof(command_list) / sizeof(command_list[0]);

void User_Init()
{
	ESP_LOGI(TAG, "User Initialization start.");
	Nut_LED(1);
	usleep(200000);
	Nut_LED(0);
}
