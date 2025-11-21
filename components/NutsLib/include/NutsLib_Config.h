/*
 * nut_config.h
 *
 *  Created on: Sep 19, 2025
 *      Author: ZonghuanWu
 */

#ifndef INC_NUTSLIB_CONFIG_H_
#define INC_NUTSLIB_CONFIG_H_

/*  DEVICE DEPENDENT SETTINGS  */
// usually remain 280k+bytes
#define NUT_BUFFER_SIZE 2000

/*  PIN DEFINITIONS  */

#define NUT_LED_PIN 4

#define NUT_IO_USER_PIN 5

#define NUT_IO1_PIN 35

#define NUT_IO2_PIN 36

#define NUT_IO3_PIN 37

#define NUT_I2C_SCL_PIN 18
#define NUT_I2C_SDA_PIN 17

#define NUT_SPI_SCK_PIN 11
#define NUT_SPI_MISO_PIN 12
#define NUT_SPI_MOSI_PIN 13
#define NUT_SPI_CS_PIN 10

#define NUT_UART_TX_PIN 9
#define NUT_UART_RX_PIN 8

/*  ALTERNATE FUNCTION DEFINITIONS  */

#define NUT_I2C I2C_NUM_0

#define NUT_SPI SPI2_HOST // though used in slave mode, must be defined as SPI2_HOST

#define NUT_UART UART_NUM_1

// #define		NUT_CAN				hcan1   // CAN not supported on ESP32

#endif /* INC_NUTSLIB_CONFIG_H_ */
