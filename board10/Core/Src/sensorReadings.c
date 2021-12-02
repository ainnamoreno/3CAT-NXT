/*
 * sensorReadings.cpp
 *
 *  Created on: Apr 28, 2021
 *      Author: guifre
 */

#include "sensorReadings.h"
//#include "main.c"

void acquireTemp(I2C_HandleTypeDef *hi2c){
	Temperatures temperatures_local;
	int i;
	HAL_StatusTypeDef ret;
	uint8_t buf[12];
	int16_t val;
	uint8_t REG_TEMP = 0x00;
	float temp_c; //defineixo float però no sé si es pot guardar al tipus Temperatures.raw
	uint8_t ADDR[7] = {/*Adreça 1, Adreça 2, etc*/};//adreces deks diferents sensors de temperatura
	for(i=1; i<=8; i++){
		// Tell TMP102 that we want to read from the temperature register
		buf[0] = REG_TEMP;
		ret = HAL_I2C_Master_Transmit(&hi2c, ADDR[i], buf, 1, 1000);
		if ( ret != HAL_OK ) {
			//strcpy((char*)buf, "Error Tx\r\n");
		} else {
			  // Read 2 bytes from the temperature register
			  ret = HAL_I2C_Master_Receive(&hi2c, ADDR[i], buf, 2, 1000);
			  if ( ret != HAL_OK ) {
				  //strcpy((char*)buf, "Error Rx\r\n");
			  } else {
				  //Combine the bytes
				  val = ((int16_t)buf[0] << 4) | (buf[1] >> 4);

				  // Convert to 2's complement, since temperature can be negative
				  if ( val > 0x7FF ) {
					  val |= 0xF000;
				  }
				  // Convert to float temperature value (Celsius)
				  temp_c = val * 0.0625;
				  switch(i) {
				  case 1:
					  temperatures_local.fields.temp1 = temp_c;
					  break;
				  case 2:
					  temperatures_local.fields.temp2 = temp_c;
					  break;
				  case 3:
					  temperatures_local.fields.temp3 = temp_c;
					  break;
				  case 4:
					  temperatures_local.fields.temp4 = temp_c;
					  break;
				  default:
					  break;
				  }
			  }
		}
	}
	Flash_Write_Data(0x08008014, temperatures_local.raw, sizeof(temperatures_local));
}

void acquireVoltage(){
	Voltages voltages;

}

void acquireCurrents(){
	Currents currents;
}

void sensorReadings(I2C_HandleTypeDef *hi2c){
	acquireTemp(&hi2c);
	acquireVoltage();
	acquireCurrents();
}
