//IMU I2C driver header
//Authors: Samuel Breslin, Kathrine Gonzalez
//11/20/2023


/*
 * preprocessor dirrectives used to avoid
 * the same header file being included multiple
 * times in the came .cpp file
 */
#ifndef IMU_I2C_DRIVER_H
#define IMU_I2C_DRIVER_H

//include the HAL libraries, this allows us to use the
#include "stm32l4xx_hal.h"
#include <stdbool.h>
//Defines **note defines are not variables**

// address is shifted one bit as the last bit is the read right bit
#define IMU_I2C_ADDR (1101000 << 1)

//registers
#define WHO_AM_I 0x75

#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

#define INT_CONFIG 0x38

#define IMU_PART_ID  0x68
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

//struct for the IMU
typedef struct {

	//I2C hanndle;
	I2C_HandleTypeDef *i2cHandle;

	//accelaration Data
	float accelData[3];

	//Gyro Data
	float gyroData[3];
} IMU;


uint8_t IMU_INIT(IMU *dev, I2C_HandleTypeDef *i2cHandle);

HAL_StatusTypeDef IMU_ReadAccel(IMU *dev);
HAL_StatusTypeDef IMU_ReadGyro(IMU *dev);

HAL_StatusTypeDef IMU_ReadRegister(IMU *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef IMU_ReadRegisters(IMU *dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef IMU_WriteRegister(IMU *dev, uint8_t reg, uint8_t *data);

#endif
