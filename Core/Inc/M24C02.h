//M24C02 I2C driver header
//Authors: Samuel Breslin, Joshua Kwak, Ahmed Sattar
//11/21/2023


/*
 * preprocessor dirrectives used to avoid
 * the same header file being included multiple
 * times in the came .cpp file
 */
#ifndef M24C02_I2C_DRIVER_H
#define M24C02_I2C_DRIVER_H

//include the HAL libraries, this allows us to use the
#include "stm32l4xx_hal.h"

//Defines **note defines are not variables**

// address is shifted one bit as the last bit is the read right bit
#define M2402_I2C_ADDR (1010000 << 1)



// data format
#define P_ADDR 0x00
#define I_ADDR 0x04 //Integral Constant
#define D_ADDR 0x08 //Derivative Constant
#define O_ADDR 0x12 //odometer
#define R_ADDR 0x16 //regen constant
#define S_ADDR 0x20 //speed constant

//setup for setting up memory
struct memory{

	int id = 0;
	uint8 addr = 0x00;
	int size = 0;
	char name[] = "x";

};

struct memory memData[] =
{
struct memory p = {.id = 0 , .addr = 0x00 , .size = 4 , .name = "potential constant"}
struct memory i = {.id = 1 , .addr = 0x04 , .size = 4 , .name = "integral constant"}
struct memory d = {.id = 2 , .addr = 0x08 , .size = 4 , .name = "derivative"}
struct memory o = {.id = 3 , .addr = 0x12 , .size = 4 , .name = "odometer"}
struct memory r = {.id = 4 , .addr = 0x16 , .size = 4 , .name = "regen constant"}
struct memory o = {.id = 5 , .addr = 0x20 , .size = 4 , .name = "speed constant"}
}





//struct for the IMU
typedef struct {

	//I2C handle;
	I2C_HandleTypeDef *i2cHandle;

	//acceleration Data
	float P;
	float I;
	float D;
	float O;
	float R;
	float S;


} M24C02;

HAL_StatusTypeDef M24C02_INIT(IMU *dev,I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef M24C02_ReadALL(IMU *dev, float *data);
HAL_StatusTypeDef M24C02_UpdateOne(IMU *dev, char selection, float newVal);
HAL_StatusTypeDef M24C02_TickOdometer(IMU *dev);

//low level functions
HAL_StatusTypeDef M24C02_ReadRegister(IMU *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef M24C02_ReadRegisters(IMU *dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef M24C02_WriteRegister(IMU *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef M24C02_WriteRegisters(IMU *dev, uint8_t reg, uint8_t *data, uint8_t length);

#endif
