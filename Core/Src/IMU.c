//IMU I2C driver
//Authors: Samuel Breslin, Kathrine Gonzalez
//11/20/2023

#include "IMU.h"
#include <stdbool.h>

uint8_t IMU_INIT(IMU *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle    = i2cHandle;

	dev->accelData[0] = 0.0f;
	dev->accelData[1] = 0.0f;
	dev->accelData[2] = 0.0f;

	dev->gyroData[0] = 0.0f;
	dev->gyroData[1] = 0.0f;
	dev->gyroData[2] = 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;
	status = IMU_ReadRegister(dev, WHO_AM_I, &regData);
	errNum += (status != HAL_OK);
	if(WHO_AM_I != 0x68){
		return(255);
	}

	regData = 0x00; //0x00 -> 250d/s, 0x08 -> 500d/s , 0x10->1000d/s, 0x18->2000d/s
	status = IMU_WriteRegister(dev, GYRO_CONFIG, &regData);
	errNum += (status != HAL_OK);

	regData = 0x08; //0x00 -> 2g, 0x08 -> 4g , 0x10->8g, 0x18->16g
	status = IMU_WriteRegister(dev, ACCEL_CONFIG, &regData);
	errNum += (status != HAL_OK);

	regData = 0x01; //0x00 -> 2g, 0x08 -> 4g , 0x10->8g, 0x18->16g
	status = IMU_WriteRegister(dev, INT_CONFIG, &regData);
	errNum += (status != HAL_OK);

	return(errNum);
}





HAL_StatusTypeDef ReadRegister(IMU *dev, uint8_t reg, uint8_t *data){
	HAL_I2C_Mem_Read(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ReadRegisters(IMU *dev, uint8_t reg, uint8_t *data, uint8_t length){
	HAL_I2C_Mem_Read(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef WriteRegister(IMU *dev, uint8_t reg, uint8_t *data){
	HAL_I2C_Mem_Write(dev->i2cHandle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
