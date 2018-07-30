
#ifndef EEPROM_H
#define EEPROM_H

#include "stm32f1xx_hal.h"

typedef enum _status
{
	EEPROM_WRITE_SUCCESS,
	EEPROM_WRITE_ERROR,
	EEPROM_READ_SUCCESS,
	EEPROM_READ_ERROR,
	EEPROM_NONE

}EEPROM_STATUS;

#define AT24C0X_ENABLE 1

#if AT24C0X_ENABLE==1

	#define AT24C0X_ADDR 0xA0
	#define AT24C0X_READ_ADDR 0XA1  	/* low bit is 0, read */
	#define AT24C0X_WRITE_ADDR 0XA0  	/* low bit is 1, write */
	#define AT24C0X_SIZE 256					/* AT24C01-1k bit 128B 16page, AT24C02-2k bit 256B 32page*/
	
	EEPROM_STATUS AT24_write_byte(I2C_HandleTypeDef *hi2c, uint8_t* pData, uint16_t Size);
	EEPROM_STATUS AT24_write_page(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);

	EEPROM_STATUS AT24_read_byte(I2C_HandleTypeDef *hi2c, uint8_t dataAddr, uint8_t* pData);
	EEPROM_STATUS AT24_read_bytes(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
	EEPROM_STATUS AT24_read_current(I2C_HandleTypeDef *hi2c, uint8_t *pData);

#endif

#endif
