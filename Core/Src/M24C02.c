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

	uint8_t addressArray[] = {P_ADDR, I_ADDR, D_ADDR, O_ADDR, R_ADDR, S_ADDR};
	uint8_t tempBytes;
	float tempFloat;

	for(i = 0; i < sizeof(addressArray); i++){
		HAL_StatusTypeDef readStatus = M24C02_ReadRegisters(dev, addressArray[i], tempBytes, 4);
		if(readStatus != HAL_OK){
			strcpy((char*)buf, "Error: Read Error"
			break;

		} else {
			Bytes_To_Float(tempBytes, tempFloat);
			data[i] = tempFloat;
		}
	}




}
HAL_StatusTypeDef M24C02_UpdateOne(M24C02 *dev, char selection, float newVal){
	
	uint8_t reg;
	bool valid = true;

	switch(selection){
	
		case 'o': //Odometer
		case 'O':
			reg = O_ADDR;
			break;
			
		case 's': //Speed
		case 'S':
			reg = S_ADDR;
			break;
			
		case 'p': //
		case 'P':
			reg = P_ADDR;
			break;
			
		case 'i': //Integral
		case 'I':
			reg = I_ADDR;
			break;
			
		case 'd': //Derivative
		case 'D':
			reg = D_ADDR;
			break;
			
		case 'r': //Regen
		case 'R':
			reg = R_ADDR;
			break;
			
			
		default;
		strcpy((char*)buf, "Error: Not Valid Selection");
		valid = false;
			break;

		if(valid){
			uint8_t dataBytes;

			Float_To_Bytes(newVal, dataBytes);

			HAL_StatusTypeDef writeStatus = M24C02_WriteRegisters(dev, reg, dataBytes, 4);

			if(writeStatus != HAL_OK){
				strcpy((char*)buf, "Error: Write Error");
			}
		}
			
	
	
	}

}
HAL_StatusTypeDef M24C02_TickOdometer(M24C02 *dev){

	float floatVal;
	uint8_t tempData[4];


	HAL_StatusTypeDef readStatus = M24C02_ReadRegisters(dev, O_ADDR, tempData, 4);
	if (readStatus != HAL_OK){
		Bytes_To_Float(tempData, floatVal);
		floatVal++;
		Float_To_Bytes(floatVal, tempData);

		HAL_StatusTypeDef writeStatus = M24C02_WriteRegisters(dev, O_ADDR, tempData, 4);
		if (writeStatus != HAL_OK){
			strcpy((char*)buf, "Error: Write Error");
		}

	} else {
		strcpy((char*)buf, "Error: Read Error");
	}


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
