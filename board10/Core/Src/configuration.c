/*!
 * \file      configuration.c
 *
 * \brief     It contains the initializing functions and all the functions that check
 * 			  the satellite state
 *
 *
 * \created on: 01/10/2021
 *
 * \author    Pol Simon
 *
 * \author    David Reiss
 *
 * \modified on: 01/03/2022
 */

#include "configuration.h"

/**************************************************************************************
 *                                                                                    *
 * Function:  checkbatteries                                                 		  *
 * --------------------                                                               *
 * Checks the current battery level	and stores it in the NVM						  *
 *                                                                                    *
 * hi2c: I2C to read battery capacity							    				  *
 *															                          *
 * returns: Nothing									                              	  *
 *                                                                                    *
 **************************************************************************************/
void checkbatteries(I2C_HandleTypeDef *hi2c) {
	uint8_t percentage;
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Master_Transmit(hi2c, BATTSENSOR_ADDR, (uint8_t*) 0x06, 1,
			500); //we want to read from the register 0x06
	if (ret != HAL_OK) {

	} else {
		HAL_I2C_Master_Receive(hi2c, BATTSENSOR_ADDR, &percentage, 1, 500);
	}
	Write_Flash(BATT_LEVEL_ADDR, &percentage, 1);
}

/**************************************************************************************
 *                                                                                    *
 * Function:  deployment                                               		  		  *
 * --------------------                                                               *
 * Induces a current through a resistor in order to burn the nylon wire and deploy	  *
 * the comms antenna. After that, the function writes delpoyment_state = true in	  *
 * the memory																		  *
 *																					  *
 *  hi2c: I2C to read temperatures in system_state()			    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void deployment(I2C_HandleTypeDef *hi2c) {
	bool deployment = false;
	while (system_state(&hi2c) == 0) {
		/*Give high voltage to the resistor to burn the wire, TBD TIME AND VOLTAGE*/
	}
	deployment = true;

	Write_Flash(DEPLOYMENT_STATE_ADDR, &deployment, 1); /*Must be stored in FLASH memory in order to keep it if the system is rebooted*/
}

/**************************************************************************************
 *                                                                                    *
 * Function:  deploymentRF                                               	  		  *
 * --------------------                                                               *
 * Induces a current through a resistor in order to burn the nylon wire and deploy	  *
 * the PL2 antenna. After that, the function writes delpoymentRF_state = true in the  *
 * memory																		  	  *
 *																					  *
 *  hi2c: I2C to read temperatures in system_state()			    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void deploymentRF(I2C_HandleTypeDef *hi2c) {

}

/**************************************************************************************
 *                                                                                    *
 * Function:  check_position                                               	  		  *
 * --------------------                                                               *
 * With the SPG4 file, checks if the satellite is in the contact range with GS  	  *
 *																					  *
 *  No input													    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void check_position() {
	//Region of contact with GS => Write_Flash(COMMS_STATE_ADDRESS, TRUE, 1);
}

/**************************************************************************************
 *                                                                                    *
 * Function:  init			                                               	  		  *
 * --------------------                                                               *
 * Simulates the "INIT" state: detumbling and deployment of the antennas. If all	  *
 * goes as expected, the next state is IDLE											  *
 *																					  *
 *  hi2c: I2C to read temperatures in system_state()			    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void init(I2C_HandleTypeDef *hi2c) {
	bool deployment_state, deploymentRF_state;
	Read_Flash(DEPLOYMENT_STATE_ADDR, &deployment_state, 1); //read the indicator of the deployment of comms antenna
	Read_Flash(DEPLOYMENTRF_STATE_ADDR, &deploymentRF_state, 1); //read the indicator of the deployment of PL2 antenna
	uint8_t currentState;
	if (!system_state(&hi2c))
		currentState = CONTINGENCY;
	else {
		if (!deployment_state)
			deployment(&hi2c);
		//Just in the PocketQube with the RF antenna
		if (!deploymentRF_state)
			deploymentRF(&hi2c);

		// thread de adcs
		detumble(&hi2c);
		currentState = CHECK;
	}
	Write_Flash(CURRENT_STATE_ADDR, &currentState, sizeof(currentState));
}

/**************************************************************************************
 *                                                                                    *
 * Function:  initsensors                                               	  		  *
 * --------------------                                                               *
 * Initializes both gyroscope and magnetometer sensors 								  *
 *																					  *
 *  hi2c: I2C to write in the registers of the sensors			    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void initsensors(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef ret;
	//GYROSCOPE CONFIGURATION
	ret = HAL_I2C_Master_Transmit(hi2c, GYRO_ADDR, (uint8_t*) 0x1A, 1, 1000); //write in the register 0x1A
	if (ret != HAL_OK) {

	} else {
//		HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, /*data to register 0x1A*/, 1, 1000);
	}
	ret = HAL_I2C_Master_Transmit(hi2c, GYRO_ADDR, (uint8_t*) 0x1B, 1, 1000); //write in the register 0x1B
	if (ret != HAL_OK) {

	} else {
//		HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, /*data to register 0x1B*/, 1, HAL_MAX_DELAY);
	}

	//MAGNETOMETER CONFIGURATION mirar a quin registre s'ha d'escriure
//	ret = HAL_I2C_Master_Transmit(&hi2c, MAG_ADDR, /**/, 1, HAL_MAX_DELAY);

}

/**************************************************************************************
 *                                                                                    *
 * Function:  system_state                                               	  		  *
 * --------------------                                                               *
 * Checks the current battery level and different temperature sensors of the		  *
 * satellite																		  *
 *																					  *
 *  hi2c: I2C to read from the temperature sensors				    				  *
 *															                          *
 *  returns: True if everything is okay					                              *
 *  		 False if temperatures are too much high or battery low					  *
 *                                                                                    *
 **************************************************************************************/
int system_state(I2C_HandleTypeDef *hi2c) {
	uint8_t nominal, low, critical, battery_capacity;

	/*If checktemperature returns false, there are different cases which must be distinguished:
	 * 	- More than three temperature sensors are hot => start rotating the satellite
	 * 	- Battery temperature is too hot => THIS CASE MUST BE STUDIED
	 * 	- MCU temperature out of operating range => THIS CASE MUST BE STUDIED */
	if (!checktemperature(hi2c)) /*rotate_satellite*/
		;

	checkbatteries(hi2c);

	/*Read from memory the threshold NOMINAL, LOW, CRITICAL and the current BATTERY LEVEL*/
	Read_Flash(BATT_LEVEL_ADDR, &battery_capacity, 1);
	Read_Flash(NOMINAL_ADDR, &nominal, 1);
	Read_Flash(LOW_ADDR, &low, 1);
	Read_Flash(CRITICAL_ADDR, &critical, 1);
	if (battery_capacity < nominal)
		return 1;
	else if (battery_capacity < low)
		return 2;
	else if (battery_capacity < critical)
		return 3;
	else
		return 0;
}

// Una funcion que ncluya todas los posibles valores de nivel de bateria,
// retorna un valor del 0-3 dependiendo del estado de la bateria ? Aqui ya se hara la comparación

/**************************************************************************************
 *                                                                                    *
 * Function:  checktemperature                                             	  		  *
 * --------------------                                                               *
 * Checks if all the temperatures are in their corresponding ranges. This function is *
 * also responsible for activating or deactivating the battery heater				  *
 *																					  *
 *  hi2c: I2C to read from the temperature sensors				    				  *
 *															                          *
 *  returns: False if more than 3 solar panels are too hot 							  *
 *  		 True otherwise								                              *
 *  		 																		  *
 **************************************************************************************/
bool checktemperature(I2C_HandleTypeDef *hi2c) {
	Temperatures temp;
	Flash_Read_Data(TEMP_ADDR, &temp.raw, sizeof(temp));
	int i, cont = 0;
	for (i = 1; i <= 7; i++) {  //number of sensors not defined yet

		switch (i) {

		case 1:
			if (temp.fields.tempbatt <= TEMP_MIN)
				heater(1);
			else
				heater(0);
			break;

		case 2:
			if (temp.fields.temp1 > TEMP_MAX)
				cont++;
			break;

		case 3:
			if (temp.fields.temp2 > TEMP_MAX)
				cont++;
			break;

		case 4:
			if (temp.fields.temp3 > TEMP_MAX)
				cont++;
			break;

		case 5:
			if (temp.fields.temp4 > TEMP_MAX)
				cont++;
			break;

		case 6:
			if (temp.fields.temp5 > TEMP_MAX)
				cont++;
			break;

		case 7:
			if (temp.fields.temp6 > TEMP_MAX)
				cont++;
			break;

		default:
			break;
			//more cases should come as much as the final number of sensors
		}

		if (cont > 3)
			return false;
		else
			return true;
	}
}

void heater(int state) {

}

/**************************************************************************************
 *                                                                                    *
 * Function:  unixTime                                            	  		  		  *
 * --------------------                                                               *
 * Converts the time in Unix Format and returns it in an array of size 4			  *
 *																					  *
 *  sTime,sDate: read from RTC from date and time			    				      *
 *															                          *
 *  returns: uint8_t t[4] with the time in unix format								  *
 *  		 																		  *
 **************************************************************************************/
time_t PL_Time(RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate) {
	//January and February are counted as months 13 and 14 of the previous year
	time_t timestamp;
	struct tm currTime;
	currTime.tm_year = sDate->Year + 100;
    currTime.tm_mon = sDate->Month-1;
    currTime.tm_mday = sDate->Date;
	currTime.tm_hour = sTime->Hours-1;
    currTime.tm_min = sTime->Minutes;
    currTime.tm_sec = sTime->Seconds;

    timestamp = mktime(&currTime);
    return timestamp;

}
