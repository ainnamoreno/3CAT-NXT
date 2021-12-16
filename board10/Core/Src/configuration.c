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
 */
#include "configuration.h"

static const uint8_t GYRO_ADDR = 0x68 << 1; //gyroscope address, 0x68 or 0x69 depending on the SA0 pin
static const uint8_t MAG_ADDR = 0x30 << 1; //magnetometer address
static const uint8_t BATTSENSOR_ADDR = 0x34 << 1; //battery sensor address

/**************************************************************************************
 *                                                                                    *
 * Function:  checkbatteries                                                 		  *
 * --------------------                                                               *
 * Checks the current battery level													  *
 *                                                                                    *
 *  No inputs													    				  *
 *															                          *
 *  returns: Battery level								                              *
 *                                                                                    *
 **************************************************************************************/
int checkbatteries(){
	BatteryLevels batterylevel;
	double actual_level, percentage;
//	actual_level = reading_EPS; //reading_EPS is still to be defined, will be acquireVoltage()?, acquirecurrent()?
//	percentage = (actual_level/MAX_BAT_LEVEL)*100;
//	batterylevel.fields.totalbattery = (int) percentage;
	return batterylevel.fields.totalbattery;

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
void deployment(I2C_HandleTypeDef *hi2c){
	bool deployment = false;
	while (system_state(&hi2c)){
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
void deploymentRF(I2C_HandleTypeDef *hi2c){

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
void init(I2C_HandleTypeDef *hi2c){
	bool deployment_state, deploymentRF_state;
	Read_Flash(DEPLOYMENT_STATE_ADDR, &deployment_state, 1);
	Read_Flash(DEPLOYMENTRF_STATE_ADDR, &deploymentRF_state, 1);

	if(!system_state(&hi2c)) currentState = CONTINGENCY;
	else {
		if(!deployment_state)	deployment(&hi2c);
		//Just in the PocketQube with the RF antenna
		if(!deploymentRF_state) deploymentRF(&hi2c);
		detumble(&hi2c);
		currentState = IDLE;
	}
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
	ret = HAL_I2C_Master_Transmit(hi2c, GYRO_ADDR, (uint8_t*)0x1A, 1, 1000); //write in the register 0x1A
	//wait for ACK?
	if (ret != HAL_OK) {

	} else {
//		HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, /*data to register 0x1A*/, 1, 1000);
	}
	ret = HAL_I2C_Master_Transmit(hi2c, GYRO_ADDR, (uint8_t*)0x1B, 1, 1000); //write in the register 0x1B
	//wait for ACK?
	if (ret != HAL_OK) {

	} else {
//		HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, /*data to register 0x1B*/, 1, HAL_MAX_DELAY);
	}

	//MAGNETOMETER CONFIGURATION mirar a quin registre s'ha d'escriure
//	ret = HAL_I2C_Master_Transmit(&hi2c, MAG_ADDR, /**/, 1, HAL_MAX_DELAY);
	//wait for ACK?


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
bool system_state(I2C_HandleTypeDef *hi2c){
	if(checkbatteries() < LOW) return false;
	else if(checkbatteries() < NOMINAL) {

	}

	//checkbatteries();
	if (!checktemperature(&hi2c)) return false;
	//EPS_state(); ???
	//illumination_state(); ???
	return true;
}

/**************************************************************************************
 *                                                                                    *
 * Function:  checktemperature                                             	  		  *
 * --------------------                                                               *
 * Checks if all the temperatures are in their corresponding ranges. This function    *
 * also responsible for activating or deactivating the battery heater				  *
 *																					  *
 *  hi2c: I2C to read from the temperature sensors				    				  *
 *															                          *
 *  returns: False if more than 3 solar panels are too hot or battery temperature 	  *
 *  		 is too low																  *
 *  		 True otherwise								                              *
 *  		 																		  *
 **************************************************************************************/
bool checktemperature(I2C_HandleTypeDef *hi2c){
	Temperatures temp;
	Flash_Read_Data(0x08008014, temp.raw, sizeof(temp));
	int i, cont = 0;
	for (i=1; i<=7; i++){  //number of sensors not defined yet

		switch(i) {

		case 1:
			if (temp.fields.tempbatt <= TEMP_MIN) heater(1);
			else heater(0);
		break;

		case 2:
			if (temp.fields.temp1 > TEMP_MAX) cont ++;
		break;

		case 3:
			if (temp.fields.temp2 > TEMP_MAX) cont ++;
		break;

		case 4:
			if (temp.fields.temp3 > TEMP_MAX) cont ++;
		break;

		case 5:
			if (temp.fields.temp4 > TEMP_MAX) cont ++;
		break;

		case 6:
			if (temp.fields.temp5 > TEMP_MAX) cont ++;
		break;

		case 7:
			if (temp.fields.temp6 > TEMP_MAX) cont ++;
		break;

		default: break;
		//more cases should come as much as the final number of sensors
		}

	if (cont > 3) return false;
	else return true;
	}
}

void heater(int state){

}

/**************************************************************************************
 *                                                                                    *
 * Function:  checkmemory	                                             	  		  *
 * --------------------                                                               *
 * Since there will be enough space in the flash memory to store the data coming	  *
 * from the payloads, this function must decide if the photo/spectogram already 	  *
 * stored in memory can be overwritten												  *
 *																					  *
 *  hi2c: I2C to read from the temperature sensors				    				  *
 *															                          *
 *  returns: False if more than 3 solar panels are too hot or battery temperature 	  *
 *  		 is too low																  *
 *  		 True otherwise								                              *
 *  		 																		  *
 **************************************************************************************/
bool checkmemory(){

}








