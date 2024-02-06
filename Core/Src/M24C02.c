//M24C02 I2C driver
//Authors: Samuel Breslin, Kathrine Gonzalez, Joshua Kwak, Ahmed Sattar
//11/21/2023

#include "M24C02.h"


uint8_t M24C02_INIT(M24C02 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle    = i2cHandle;

	return(1);
}

void Float_To_Bytes(float val, uint8_t* bytes){ //Converts float to a 4 byte array. Pass the float and an pointer to the save location of the bytes.

	union u //Creates a shared memory space of the largest item (4 bytes).
	{
		float tempFloat; //Both items are saved in the same memory space concurrently.
		uint8_t bytesArray[4];
	};

	u.tempFloat = val;

	memcpy(bytes, u.tempFloat, 4); //Copies the data from the float value to the bytes
}

void String_To_Bytes(char a[] , uint8_t* bytes){ //Converts a c-string to a series of bytes. Pass a char array and a pointer to the save location of the bytes.

	union u
	{
		char tempString[];
		uint8_t bytesArray[sizeof(a[])];
	};
	memcpy(bytes, u.tempString, sizeof(a[])
}

void Int_To_Bytes(int val, uint8_t* bytes){ //Converts an int to bytes. Pass an int and a pointer to the save location of the bytes.

	union u
	{
		int tempInt;
		uint8_t bytesArray[1];
	};
	memcpy(bytes, u.tempInt, 1);
}




void Bytes_To_Float(byte* bytes, float &val){ //Converts bytes to a float. Pass the byte pointer and the call-by-reference float variable.
	union u
	{
		float tempFloat;
		uint8_t bytesArray[4];
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

HAL_StatusTypeDef M24C02_FetchMemData(M24C02 *dev, uint8_t *bytes[]){

	for(i = 0 ; i < sizeof(memData) ; i++){

		uint8_t IDBytes[];
		uint8_t NameBytes[];

		int tempID = memData[i].id;
		Int_To_Bytes(tempID, IDBytes);

		char tempString = memData[i].name;
		String_To_Bytes(tempString, NameBytes);

		//Send data to GUI
	}
	//Send terminating cmd

}

HAL_StatusTypeDef M24C02_UpdateMem(M24C02 *dev, int ID, uint8_t* newVal){

	bool pointFound = false;

	int arrSize = sizeof(memData);
	int i = arrySize/2;

	while(!pointFound){

		if(memData[i].id = ID){
			int desID = i;
			pointFound = true;
			break;
		} else if(memData[i].id > ID) {
			i = i/2;
		}else {
			i = i;
		}


	}

}

HAL_StatusTypeDef M24C02_UpdateOne(M24C02 *dev, char selection, float newVal){//Function to write the data to an individual memory space based on the requested input.
	
	uint8_t reg;
	bool valid = true;

	switch(selection){ //Switches based on the selected input and will set the register variable(reg) to the staring addess value.
	
		case 'o': //Odometer
		case 'O':
			reg = O_ADDR;
			break;
			
		case 's': //Speed
		case 'S':
			reg = S_ADDR;
			break;
			
		case 'p': //Potential
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
			uint8_t dataBytes[4];

			Float_To_Bytes(newVal, dataBytes);

			HAL_StatusTypeDef writeStatus = M24C02_WriteRegisters(dev, reg, dataBytes, 4); //As long as the writing is HAL_OK, then it will execute properly.

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
