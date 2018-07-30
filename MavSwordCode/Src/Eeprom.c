
#include "Eeprom.h"

#if AT24C0X_ENABLE==1
	
	
	/*** 写一个字节数据 devAddr + dataAddr + data */
	EEPROM_STATUS AT24_write_byte(I2C_HandleTypeDef *hi2c, uint8_t* pData, uint16_t Size)
	{
		HAL_StatusTypeDef st = HAL_ERROR;
		
		if(Size!=2 || *pData>=AT24C0X_SIZE) return EEPROM_WRITE_ERROR; /* write a byte: devADDR+wordAddr+data		*/
		
		st = HAL_I2C_Master_Transmit(hi2c, AT24C0X_WRITE_ADDR, pData, 2, 72000000ul);
		if(st==HAL_OK)
			return EEPROM_WRITE_SUCCESS;
		else
			return EEPROM_WRITE_ERROR;
	}
	
	/* write page data: 8 byte a page  */
	EEPROM_STATUS AT24_write_page(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
	{
		HAL_StatusTypeDef st = HAL_ERROR;
		
		if(Size>8 || *pData>=AT24C0X_SIZE) return EEPROM_WRITE_ERROR; 
		if( (*pData%8)!=0 ) return EEPROM_WRITE_ERROR;
		
		st = HAL_I2C_Master_Transmit(hi2c, AT24C0X_WRITE_ADDR, pData, Size, 7200000ul);
		if(st==HAL_OK)
			return EEPROM_WRITE_SUCCESS;
		else
			return EEPROM_WRITE_ERROR;
	}

	/* read a byte. W: devAddr + dataAddr, R: devAddr */
	EEPROM_STATUS AT24_read_byte(I2C_HandleTypeDef *hi2c, uint8_t dataAddr, uint8_t* pData)
	{
		HAL_StatusTypeDef st = HAL_ERROR;
		
		if(0==pData || dataAddr>AT24C0X_SIZE )  return EEPROM_WRITE_ERROR; 
		
		st = HAL_I2C_Master_Transmit(hi2c, AT24C0X_WRITE_ADDR, &dataAddr, 1, 7200000ul);
		st = HAL_I2C_Master_Receive(hi2c, AT24C0X_READ_ADDR, pData, 1, 7200000ul);
		if(st==HAL_OK)
			return EEPROM_WRITE_SUCCESS;
		else
			return EEPROM_WRITE_ERROR;
	}
	
	/* read bytes. devAddr + dataAddr + devAddr, return data */
	EEPROM_STATUS AT24_read_bytes(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
	{
		HAL_StatusTypeDef st = HAL_ERROR;
		
		if(pData[1]>=AT24C0X_SIZE || Size>(AT24C0X_SIZE-pData[1]) )  return EEPROM_WRITE_ERROR; 
		
		st = HAL_I2C_Master_Receive(hi2c, AT24C0X_READ_ADDR, pData, Size, 7200000ul);
		if(st==HAL_OK)
			return EEPROM_WRITE_SUCCESS;
		else
			return EEPROM_WRITE_ERROR;
	}
	
	/* read current byte, hardware auto increment 1 */
	EEPROM_STATUS AT24_read_current(I2C_HandleTypeDef *hi2c, uint8_t *pData)
	{
		HAL_StatusTypeDef st = HAL_ERROR;
		
		if(0==pData) return EEPROM_WRITE_ERROR; 
		
		st = HAL_I2C_Master_Receive(hi2c, AT24C0X_READ_ADDR, pData, 1, 7200000ul);
		if(st==HAL_OK)
			return EEPROM_WRITE_SUCCESS;
		else
			return EEPROM_WRITE_ERROR;
	}

#endif

