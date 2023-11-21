//M24C02 I2C driver
//Authors: Samuel Breslin, Kathrine Gonzalez
//11/21/2023

#include "M24C02.h"


uint8_t M24C02_INIT(IMU *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle    = i2cHandle;

	return(1);
}

HAL_StatusTypeDef M24C02_ReadALL(IMU *dev, float *data){

}
HAL_StatusTypeDef M24C02_UpdateOne(IMU *dev, char selection, float newVal){

}
HAL_StatusTypeDef M24C02_TickOdometer(IMU *dev){

}

//low level functions
HAL_StatusTypeDef M24C02_ReadRegister(IMU *dev, uint8_t reg, uint8_t *data){
	HAL_I2C_Mem_Read(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_ReadRegisters(IMU *dev, uint8_t reg, uint8_t *data, uint8_t length){
	HAL_I2C_Mem_Read(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_WriteRegister(IMU *dev, uint8_t reg, uint8_t *data){
	HAL_I2C_Mem_Write(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef M24C02_WriteRegisters(IMU *dev, uint8_t reg, uint8_t *data, uint8_t length){
	HAL_I2C_Mem_Write(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}
