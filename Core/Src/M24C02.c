//M24C02 I2C driver
//Authors: Samuel Breslin, Kathrine Gonzalez
//11/21/2023

#include "M24C02.h"


uint8_t M24C02_INIT(M24C02 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle    = i2cHandle;

	return(1);
}

void Float_To_Bytes(float val, byte* bytes){ //Converts float to a 4 byte array. Pass the float and where the bytes should be converted.

	union u
	{
		float tempFloat;
		byte bytesArray[4];
	};

	u.tempFloat = val;

	memcpy(bytes, u.tempFloat, 4);
}

void Bytes_To_Float(byte* bytes, float &val){ //Converts bytes to a float. Pass the byte pointer and the call-by-reference float variable.
	union u
	{
		float tempFloat;
		byte bytesArray[4];
	};
	u.bytesArray = bytes;
	
	memcpy(val, u.bytesArray, 4);
}

HAL_StatusTypeDef M24C02_ReadALL(M24C02 *dev, float *data){

}
HAL_StatusTypeDef M24C02_UpdateOne(M24C02 *dev, char selection, float newVal){
	
	switch(selection){
	
		case 'o':
		case 'O':
			
			M24C02_WriteRegisters(dev, )
			
			break;
			
		case 's':
		case 'S':
			
			break;
			
		case 'p':
		case 'P':
			
			break;
			
		case 'i':
		case 'I':
			
			break;
			
		case 'd':
		case 'D':
			
			break;
			
		default;
		
			break;
			
	
	
	}

}
HAL_StatusTypeDef M24C02_TickOdometer(M24C02 *dev){

}

//low level functions
HAL_StatusTypeDef M24C02_ReadRegister(M24C02 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_ReadRegisters(M24C02 *dev, uint8_t reg, uint8_t *data, uint8_t length){
	return HAL_I2C_Mem_Read(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_WriteRegister(M24C02 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_WriteRegisters(M24C02 *dev, uint8_t reg, uint8_t *data, uint8_t length){
	return HAL_I2C_Mem_Write(dev->i2cHandle, M24C02_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}
