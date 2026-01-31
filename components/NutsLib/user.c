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

/* mbedTLS AES */
#include "mbedtls/aes.h"
#include "aes/esp_aes.h"
#include "hal/aes_hal.h"
#include "aes/esp_aes_internal.h"
#include "soc/hwcrypto_reg.h"
#include "hal/aes_types.h"

mbedtls_aes_context aes_ctx;
#define KEY_LENGTH 128

NutStatus_e AES_SetEncryptionKey(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
								 uint32_t result_buffer_MAX_size)
{
	*result_length = 0;
	if (mbedtls_aes_setkey_enc(&aes_ctx, received_data_ptr, KEY_LENGTH))
		return NUT_ERROR;
	return NUT_OK;
}

NutStatus_e AES_SetDecryptionKey(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
								 uint32_t result_buffer_MAX_size)
{
	*result_length = 0;
	if (mbedtls_aes_setkey_dec(&aes_ctx, received_data_ptr, KEY_LENGTH))
		return NUT_ERROR;
	return NUT_OK;
}

NutStatus_e AES_Encrypt(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
						uint32_t result_buffer_MAX_size)
{
	*result_length = 16;

	// int status = mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, received_data_ptr, result_buffer_ptr);

	Nut_LED(1);
	Nut_Trigger_Set();

	// Acquire AES hardware and enter critocal section
	esp_aes_acquire_hardware();

	// Set key and read back key in hardware
	aes_ctx.key_in_hardware = 0;
	aes_ctx.key_in_hardware = aes_hal_setkey(aes_ctx.key, aes_ctx.key_bytes, MBEDTLS_AES_ENCRYPT);
	// aes_hal_mode_init(ESP_AES_BLOCK_MODE_ECB);

	// disable DMA
	REG_WRITE(AES_DMA_ENABLE_REG, 0);

	// initialize AES hardware in ECB mode
	// REG_WRITE(AES_MODE_REG, 0x00); // AES-128 Encrypt (already done in aes_hal_setkey)
	// Set key (already done in aes_hal_setkey)
	// Copy text to input registers
	uint32_t temp32u;
	for (int i = 0; i < AES_BLOCK_WORDS; i++)
	{
		memcpy(&temp32u, received_data_ptr + 4 * i, 4);
		REG_WRITE(AES_TEXT_IN_BASE + i * 4, temp32u);
	}

	// Since you asked
	Nut_Trigger_Clear();

	// Start encryption
	REG_WRITE(AES_TRIGGER_REG, 1);

	// Wait until done
	while (REG_READ(AES_STATE_REG) != ESP_AES_STATE_IDLE)
		;

	// aes_hal_transform_block(received_data_ptr, result_buffer_ptr);

	// int status = esp_aes_block(&aes_ctx, received_data_ptr, result_buffer_ptr);

	// Since you asked
	Nut_Trigger_Set();

	// Read back output registers
	for (size_t i = 0; i < AES_BLOCK_WORDS; i++)
	{
		temp32u = REG_READ(AES_TEXT_OUT_BASE + (i * 4));
		/* Memcpy to avoid potential unaligned access */
		memcpy(result_buffer_ptr + i * 4, &temp32u, sizeof(temp32u));
	}

	// release hardware
	esp_aes_release_hardware();

	Nut_Trigger_Clear();
	Nut_LED(0);

	return NUT_OK;
}

// NutStatus_e AES_Encrypt(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
// 						uint32_t result_buffer_MAX_size)
// {
// 	*result_length = 16;
// 	// Nut_Quiet();
// 	Nut_LED(1);
// 	Nut_Trigger_Set();
// 	int status = mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, received_data_ptr, result_buffer_ptr);
// 	Nut_Trigger_Clear();
// 	Nut_LED(0);
// 	// Nut_Unquiet();
// 	if (status == 0)
// 	{
// 		return NUT_OK;
// 	}
// 	else
// 	{
// 		return NUT_ERROR;
// 	}
// }

NutStatus_e AES_Decrypt(uint8_t *received_data_ptr, uint32_t received_data_length, uint8_t *result_buffer_ptr, uint32_t *result_length,
						uint32_t result_buffer_MAX_size)
{
	*result_length = 16;
	// Nut_Quiet();
	Nut_LED(1);
	Nut_Trigger_Set();
	int status = mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_DECRYPT, received_data_ptr, result_buffer_ptr);
	Nut_Trigger_Clear();
	Nut_LED(0);
	// Nut_Unquiet();
	if (status == 0)
	{
		return NUT_OK;
	}
	else
	{
		return NUT_ERROR;
	}
}

/* User command */
// @formatter:off
NutAction_t command_list[] = {
	{.command = 0x0001, .function = Echo},
	/* mbedTLS AES */
	{.command = 0x0100, .function = AES_SetEncryptionKey},
	{.command = 0x0101, .function = AES_SetDecryptionKey},
	{.command = 0x0102, .function = AES_Encrypt},
	{.command = 0x0103, .function = AES_Decrypt},
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
